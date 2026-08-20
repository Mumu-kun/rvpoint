// test_openmp_rvv_scaling.cpp
// Verifying Hardware Multi-Core OpenMP Scaling for Orange Pi RV2 (SpacemiT K1 8 Cores)
#include "io/simple_pcd_loader.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <omp.h>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

int main(int argc, char **argv) {
  std::string file_path = "data/pcd_compressed/0000000080.pcd";
  std::vector<PointXYZ> raw_pts;
  if (loadPCD(file_path, raw_pts) <= 0) return 1;
  const size_t n = raw_pts.size();

  std::vector<float> rx(n), ry(n), rz(n);
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
  for (size_t i = 0; i < n; ++i) {
    rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z;
    min_x = std::min(min_x, rx[i]); max_x = std::max(max_x, rx[i]);
    min_y = std::min(min_y, ry[i]); max_y = std::max(max_y, ry[i]);
  }

  const float cell_size = 0.20f;
  const float inv_cell = 1.0f / cell_size;
  const int cols = static_cast<int>(std::ceil((max_x - min_x) * inv_cell)) + 1;
  const int rows = static_cast<int>(std::ceil((max_y - min_y) * inv_cell)) + 1;
  const size_t total_cells = cols * rows;

  std::cout << "====================================================================================\n";
  std::cout << "        HARDWARE MULTI-CORE OPENMP SCALING VERIFICATION (Orange Pi RV2)              \n";
  std::cout << "        Dataset: " << n << " Points | Grid: " << cols << "x" << rows << " (" << total_cells << " cells)\n";
  std::cout << "====================================================================================\n";
  std::cout << " Threads      Ingest & Index       Ground Separate        Total Compute       Speedup\n";
  std::cout << "------------------------------------------------------------------------------------\n";

  const std::vector<int> thread_counts = {1, 2, 4, 8};
  double baseline_ms = 0.0;

  for (int num_threads : thread_counts) {
    omp_set_num_threads(num_threads);

    // Warmup
    std::vector<uint32_t> cell_indices(n);
    std::vector<float> grid_min_z(total_cells * num_threads, 1e9f);
    std::vector<uint32_t> grid_count(total_cells * num_threads, 0);

    auto t0 = Clock::now();

    // 1. Multi-Threaded RVV Ingest
    #pragma omp parallel
    {
      int tid = omp_get_thread_num();
      float *t_min_z = grid_min_z.data() + tid * total_cells;
      uint32_t *t_cnt = grid_count.data() + tid * total_cells;

      #pragma omp for schedule(static)
      for (size_t chunk_start = 0; chunk_start < n; chunk_start += 256) {
        size_t chunk_end = std::min(chunk_start + 256, n);
        size_t i = chunk_start;
#if defined(__riscv_vector)
        while (i < chunk_end) {
          size_t vl = __riscv_vsetvl_e32m8(chunk_end - i);
          vfloat32m8_t vx = __riscv_vle32_v_f32m8(rx.data() + i, vl);
          vfloat32m8_t vy = __riscv_vle32_v_f32m8(ry.data() + i, vl);
          vuint32m8_t vc = __riscv_vfcvt_rtz_xu_f_v_u32m8(__riscv_vfmul_vf_f32m8(__riscv_vfsub_vf_f32m8(vx, min_x, vl), inv_cell, vl), vl);
          vuint32m8_t vr = __riscv_vfcvt_rtz_xu_f_v_u32m8(__riscv_vfmul_vf_f32m8(__riscv_vfsub_vf_f32m8(vy, min_y, vl), inv_cell, vl), vl);
          vuint32m8_t v_idx = __riscv_vmacc_vx_u32m8(vc, cols, vr, vl);
          __riscv_vse32_v_u32m8(cell_indices.data() + i, v_idx, vl);

          for (size_t lane = 0; lane < vl; ++lane) {
            uint32_t idx = cell_indices[i + lane];
            if (idx < total_cells) {
              float pz = rz[i + lane];
              if (pz < t_min_z[idx]) t_min_z[idx] = pz;
              t_cnt[idx]++;
            }
          }
          i += vl;
        }
#else
        for (size_t k = chunk_start; k < chunk_end; ++k) {
          int c = static_cast<int>((rx[k] - min_x) * inv_cell);
          int r = static_cast<int>((ry[k] - min_y) * inv_cell);
          uint32_t idx = r * cols + c;
          cell_indices[k] = idx;
          if (idx < total_cells) {
            if (rz[k] < t_min_z[idx]) t_min_z[idx] = rz[k];
            t_cnt[idx]++;
          }
        }
#endif
      }
    }

    // Merge thread grids
    std::vector<float> final_min_z(total_cells, 1e9f);
    std::vector<uint32_t> final_count(total_cells, 0);
    for (int t = 0; t < num_threads; ++t) {
      const float *t_min_z = grid_min_z.data() + t * total_cells;
      const uint32_t *t_cnt = grid_count.data() + t * total_cells;
      for (size_t c = 0; c < total_cells; ++c) {
        if (t_min_z[c] < final_min_z[c]) final_min_z[c] = t_min_z[c];
        final_count[c] += t_cnt[c];
      }
    }
    double t_ingest_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    // 2. Multi-Threaded Ground Separation
    auto t1 = Clock::now();
    std::vector<float> ox(n), oy(n), oz(n);
    std::vector<size_t> thread_counts_vec(num_threads, 0);

    #pragma omp parallel
    {
      int tid = omp_get_thread_num();
      size_t local_count = 0;
      size_t thread_offset = tid * (n / num_threads);

      #pragma omp for schedule(static)
      for (size_t chunk_start = 0; chunk_start < n; chunk_start += 256) {
        size_t chunk_end = std::min(chunk_start + 256, n);
        for (size_t k = chunk_start; k < chunk_end; ++k) {
          uint32_t idx = cell_indices[k];
          if (idx < total_cells && rz[k] > final_min_z[idx] + 0.20f && final_count[idx] >= 2) {
            ox[thread_offset + local_count] = rx[k];
            oy[thread_offset + local_count] = ry[k];
            oz[thread_offset + local_count] = rz[k];
            local_count++;
          }
        }
      }
      thread_counts_vec[tid] = local_count;
    }
    double t_ground_ms = std::chrono::duration<double, std::milli>(Clock::now() - t1).count();
    double total_ms = t_ingest_ms + t_ground_ms;

    if (num_threads == 1) baseline_ms = total_ms;

    std::cout << " " << std::left << std::setw(11) << (std::to_string(num_threads) + " Cores")
              << std::right << std::fixed << std::setprecision(2)
              << std::setw(12) << t_ingest_ms << " ms"
              << std::setw(18) << t_ground_ms << " ms"
              << std::setw(18) << total_ms << " ms"
              << std::setprecision(2)
              << std::setw(13) << (baseline_ms / total_ms) << "x\n";
  }

  std::cout << "====================================================================================\n";
  return 0;
}
