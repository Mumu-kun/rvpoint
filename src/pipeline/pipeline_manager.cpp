#include "pipeline/pipeline_manager.h"

#include <queue>
#include <unordered_map>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace rvpoint {

void PipelineManager::initialize(std::size_t default_capacity) {
  // 1. Resolve DAG dependency order using Kahn's algorithm
  resolve_dag_order();

  // 2. Pre-allocate buffer capacity across all data slots
  rf_.preallocate_buffers(default_capacity);

  // 3. Resolve primary input slot ID if configured
  if (!primary_input_name_.empty()) {
    primary_input_id_ = rf_.get_id(primary_input_name_);
  }

  // 4. Pre-bind argument pointers for every node in strict positional order
  for (auto& node : nodes_) {
    node.bound_args.clear();
    for (const auto& arg : node.arg_bindings) {
      node.bound_args.push_back(rf_.get_pointer(arg.id));
    }
  }

  // 5. Pre-resolve probe slot IDs for zero runtime lookups
  for (auto& pe : probes_) {
    pe.slot_id = rf_.get_id(pe.slot_name);
  }
}

void PipelineManager::resolve_dag_order() {
  const std::size_t n = nodes_.size();
  if (n <= 1) return;

  // Map each output slot to its producer node index
  std::unordered_map<uint16_t, std::size_t> slot_producers;
  for (std::size_t i = 0; i < n; ++i) {
    for (auto sid : nodes_[i].output_slots) {
      auto insert_res = slot_producers.emplace(sid.index, i);
      if (!insert_res.second) {
        throw std::runtime_error(
            "Fatal: Multiple producers detected for slot: " + rf_.name(sid));
      }
    }
  }

  // Build DAG adjacency graph and calculate in-degrees
  std::vector<std::vector<std::size_t>> adj(n);
  std::vector<std::size_t> in_degree(n, 0);

  for (std::size_t i = 0; i < n; ++i) {
    for (auto sid : nodes_[i].input_slots) {
      auto it = slot_producers.find(sid.index);
      if (it != slot_producers.end() && it->second != i) {
        adj[it->second].push_back(i);
        in_degree[i]++;
      }
    }
  }

  // Kahn's algorithm: queue nodes with no unsatisfied dependencies
  std::queue<std::size_t> q;
  for (std::size_t i = 0; i < n; ++i) {
    if (in_degree[i] == 0) {
      q.push(i);
    }
  }

  std::vector<CompiledNode> sorted;
  sorted.reserve(n);

  while (!q.empty()) {
    std::size_t curr = q.front();
    q.pop();
    sorted.push_back(std::move(nodes_[curr]));

    for (std::size_t next : adj[curr]) {
      if (--in_degree[next] == 0) {
        q.push(next);
      }
    }
  }

  // If not all nodes were scheduled, a cyclic dependency exists
  if (sorted.size() != n) {
    throw std::runtime_error("Fatal: Cyclic dependency detected in pipeline graph!");
  }

  nodes_ = std::move(sorted);
}

void PipelineManager::step(const PointCloudView& cloud) {
  if (!primary_input_id_.is_valid()) {
    throw std::runtime_error(
        "Primary input slot not configured! Call set_primary_input() before step(cloud).");
  }

  execute_frame_internal([&](RegisterFile& rf) {
    rf.get_mut<PointCloud>(primary_input_id_).copy_from(cloud);
  });
}

void PipelineManager::print_telemetry() const {
  std::cout << "\n================ PIPELINE EXECUTION TELEMETRY ================\n";
  double total_ms = 0.0;
  for (const auto& node : nodes_) {
    std::cout << "  Stage: " << std::left << std::setw(24) << node.name
              << " | Time: " << std::fixed << std::setprecision(3)
              << std::setw(7) << node.last_execution_time_ms << " ms\n";
    total_ms += node.last_execution_time_ms;
  }
  std::cout << "  ------------------------------------------------------------\n";
  std::cout << "  TOTAL PIPELINE TIME:      " << std::fixed << std::setprecision(3)
            << total_ms << " ms (" << (total_ms > 0 ? (1000.0 / total_ms) : 0.0) << " FPS)\n";
  std::cout << "==============================================================\n\n";
}

} // namespace rvpoint

