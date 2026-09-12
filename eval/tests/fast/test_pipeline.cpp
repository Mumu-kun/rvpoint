#include "include/rvpoint.h"

#include <iostream>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rvpoint;

static void test_register_file_basics() {
  std::cout << "[Test 1] RegisterFile basic slot and param registration..." << std::endl;
  RegisterFile rf;

  RegisterId s1 = rf.get_or_register_slot<PointCloud>("cloud_a");
  RegisterId s2 = rf.get_or_register_slot<PlaneModel>("plane_a");
  RegisterId p1 = rf.get_or_register_param<float>("leaf_size", 0.05f);
  RegisterId p2 = rf.get_or_register_param<int>("max_iter", 100);

  assert(s1.is_valid());
  assert(s2.is_valid());
  assert(p1.is_valid());
  assert(p2.is_valid());
  assert(s1 != s2);
  assert(p1 != p2);

  assert(rf.find("cloud_a") == s1);
  assert(rf.find("non_existent") == RegisterId{});

  // Check initial values
  assert(std::abs(rf.get_param<float>(p1) - 0.05f) < 1e-6f);
  assert(rf.get_param<int>(p2) == 100);
  assert(!rf.is_dirty());

  // Test set_param dirty tracking
  rf.set_param(p1, 0.02f);
  assert(rf.is_dirty());
  assert(rf.changed_params().size() == 1);
  assert(rf.changed_params()[0] == p1);
  assert(std::abs(rf.get_param<float>(p1) - 0.02f) < 1e-6f);

  rf.clear_dirty();
  assert(!rf.is_dirty());
  assert(rf.changed_params().empty());

  // Pre-allocation test
  rf.preallocate_buffers(1000);
  auto& cloud = rf.get_mut<PointCloud>(s1);
  assert(cloud.x.capacity() >= 1000);
  assert(cloud.n == 0);

  // Frame reset test
  cloud.push_back(1.0f, 2.0f, 3.0f);
  assert(cloud.n == 1);
  rf.reset_frame();
  assert(cloud.n == 0);
  assert(cloud.x.capacity() >= 1000); // Capacity retained!

  std::cout << "  ✓ RegisterFile basics passed." << std::endl;
}

static void test_dag_topological_sort() {
  std::cout << "[Test 2] Pipeline DAG topological sorting (Kahn's algorithm)..." << std::endl;
  PipelineManager pm;

  // Register nodes intentionally out of order:
  // Node 3 consumes "filtered_b" and produces "output_c"
  // Node 1 consumes "raw_input" and produces "filtered_a"
  // Node 2 consumes "filtered_a" and produces "filtered_b"

  std::vector<std::string> execution_order;

  pm.add_node("node_3",
      in<PointCloud>("filtered_b"),
      out<PointCloud>("output_c")
  ).kernel([&](const PointCloud& in_cloud, PointCloud& out_cloud) {
    execution_order.push_back("node_3");
    out_cloud.resize(in_cloud.n);
  });

  pm.add_node("node_1",
      in<PointCloud>("raw_input"),
      out<PointCloud>("filtered_a")
  ).kernel([&](const PointCloud& in_cloud, PointCloud& out_cloud) {
    execution_order.push_back("node_1");
    out_cloud.resize(in_cloud.n);
  });

  pm.add_node("node_2",
      in<PointCloud>("filtered_a"),
      out<PointCloud>("filtered_b")
  ).kernel([&](const PointCloud& in_cloud, PointCloud& out_cloud) {
    execution_order.push_back("node_2");
    out_cloud.resize(in_cloud.n);
  });

  pm.set_primary_input("raw_input");
  pm.initialize(256);

  // Execution order after Kahn's algorithm should be: node_1 -> node_2 -> node_3
  assert(pm.nodes().size() == 3);
  assert(pm.nodes()[0].name == "node_1");
  assert(pm.nodes()[1].name == "node_2");
  assert(pm.nodes()[2].name == "node_3");

  PointCloud raw;
  raw.push_back(1.0f, 2.0f, 3.0f);
  pm.step(raw);

  assert(execution_order.size() == 3);
  assert(execution_order[0] == "node_1");
  assert(execution_order[1] == "node_2");
  assert(execution_order[2] == "node_3");

  std::cout << "  ✓ DAG topological sort verified." << std::endl;
}

static void test_cycle_detection() {
  std::cout << "[Test 3] Pipeline cyclic dependency detection..." << std::endl;
  PipelineManager pm;

  // Node A produces slot_x from slot_y
  pm.add_node("node_A",
      in<PointCloud>("slot_y"),
      out<PointCloud>("slot_x")
  ).kernel([](const PointCloud&, PointCloud&) {});

  // Node B produces slot_y from slot_x (CYCLE: A -> B -> A)
  pm.add_node("node_B",
      in<PointCloud>("slot_x"),
      out<PointCloud>("slot_y")
  ).kernel([](const PointCloud&, PointCloud&) {});

  bool exception_caught = false;
  try {
    pm.initialize(64);
  } catch (const std::runtime_error& e) {
    exception_caught = true;
    std::cout << "  ✓ Caught expected cyclic dependency exception: " << e.what() << std::endl;
  }
  assert(exception_caught);
}

static void test_interleaved_tagged_binding() {
  std::cout << "[Test 4] Interleaved Tagged Positional Binding..." << std::endl;
  PipelineManager pm;

  // Interleave input, param, output, param in non-standard order
  pm.add_node("interleaved_kernel",
      param<float>("scale_factor", 2.5f),
      in<PointCloud>("src_cloud"),
      param<int>("offset_pts", 10),
      out<PointCloud>("dst_cloud")
  ).kernel([](float scale, const PointCloud& src, int offset, PointCloud& dst) {
    dst.resize(src.n + static_cast<std::size_t>(offset));
    for (std::size_t i = 0; i < src.n; ++i) {
      dst.x[i] = src.x[i] * scale;
      dst.y[i] = src.y[i] * scale;
      dst.z[i] = src.z[i] * scale;
    }
    for (std::size_t i = src.n; i < dst.n; ++i) {
      dst.x[i] = 0.0f;
      dst.y[i] = 0.0f;
      dst.z[i] = 0.0f;
    }
  });

  pm.set_primary_input("src_cloud");
  pm.initialize(128);

  PointCloud in_cloud;
  in_cloud.push_back(1.0f, 2.0f, 3.0f);
  pm.step(in_cloud);

  const auto& dst = pm.registers().get<PointCloud>(pm.get_id("dst_cloud"));
  assert(dst.n == 11);
  assert(std::abs(dst.x[0] - 2.5f) < 1e-6f);
  assert(std::abs(dst.y[0] - 5.0f) < 1e-6f);
  assert(std::abs(dst.z[0] - 7.5f) < 1e-6f);

  std::cout << "  ✓ Interleaved positional bindings executed accurately." << std::endl;
}

static void test_dynamic_reconfiguration_and_probes() {
  std::cout << "[Test 5] Dynamic parameter reconfiguration & declarative probes..." << std::endl;
  PipelineManager pm;

  bool reconfig_fired = false;
  float latest_configured_thresh = 0.0f;

  pm.add_node("adaptive_filter",
      in<PointCloud>("raw"),
      out<PointCloud>("out"),
      param<float>("thresh", 1.0f)
  )
  .on_reconfig<float>("thresh", [&](RegisterFile&, float new_val) {
    reconfig_fired = true;
    latest_configured_thresh = new_val;
  })
  .kernel([&](const PointCloud& src, PointCloud& dst, float thresh) {
    dst.clear();
    for (std::size_t i = 0; i < src.n; ++i) {
      if (src.x[i] >= thresh) {
        dst.push_back(src.x[i], src.y[i], src.z[i]);
      }
    }
  });

  std::size_t probed_out_count = 0;
  pm.add_probe<PointCloud>("out", [&](const PointCloud& p) {
    probed_out_count = p.n;
  });

  bool frame_started = false;
  bool frame_ended = false;
  pm.on_frame_start([&](RegisterFile&) { frame_started = true; });
  pm.on_frame_end([&](const RegisterFile&) { frame_ended = true; });

  pm.set_primary_input("raw");
  pm.initialize(128);

  PointCloud cloud;
  cloud.push_back(0.5f, 0.0f, 0.0f);
  cloud.push_back(1.5f, 0.0f, 0.0f);
  cloud.push_back(2.5f, 0.0f, 0.0f);

  // Frame 1: default thresh = 1.0 -> 2 points pass (1.5, 2.5)
  pm.step(cloud);
  assert(frame_started && frame_ended);
  assert(!reconfig_fired);
  assert(probed_out_count == 2);

  // Frame 2: dynamic update thresh = 2.0 -> triggers hook, 1 point passes (2.5)
  pm.set_param("thresh", 2.0f);
  pm.step(cloud);
  assert(reconfig_fired);
  assert(std::abs(latest_configured_thresh - 2.0f) < 1e-6f);
  assert(probed_out_count == 1);

  std::cout << "  ✓ Dynamic reconfig hooks and declarative probes verified." << std::endl;
}

int main() {
  std::cout << "=== Running Fast Tests: Production Pipeline Engine ===" << std::endl;

  test_register_file_basics();
  test_dag_topological_sort();
  test_cycle_detection();
  test_interleaved_tagged_binding();
  test_dynamic_reconfiguration_and_probes();

  std::cout << "\n>>> ALL PIPELINE ENGINE TESTS PASSED! <<<" << std::endl;
  return 0;
}

