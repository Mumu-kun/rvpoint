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

static void test_slot_lifetime_policies() {
  std::cout << "[Test 6] FrameContext SlotLifetime policies (Ephemeral, Persistent, History, External)..." << std::endl;
  FrameContext fc;

  // 1. Ephemeral slot
  SlotId ephem_id = fc.get_or_register_slot<PointCloud>("ephemeral_cloud", SlotLifetime::Ephemeral);
  auto& ephem = fc.get_mut<PointCloud>(ephem_id);
  ephem.push_back(1.0f, 2.0f, 3.0f);
  assert(ephem.n == 1);

  // 2. Persistent slot
  SlotId persist_id = fc.get_or_register_slot<int>("persistent_counter", SlotLifetime::Persistent, 42);
  assert(fc.get<int>(persist_id) == 42);
  fc.get_mut<int>(persist_id) = 100;

  // 3. History slot: registers "scan" and "scan.prev"
  SlotId hist_id = fc.get_or_register_history<PointCloud>("scan");
  SlotId hist_prev_id = fc.get_id("scan.prev");
  assert(hist_id.is_valid() && hist_prev_id.is_valid());
  auto& scan_curr = fc.get_mut<PointCloud>(hist_id);
  auto& scan_prev = fc.get_mut<PointCloud>(hist_prev_id);
  assert(scan_curr.n == 0 && scan_prev.n == 0);

  scan_curr.push_back(10.0f, 20.0f, 30.0f); // Frame 0 data in "scan"

  // 4. External slot
  PointCloud external_cloud;
  external_cloud.push_back(99.0f, 99.0f, 99.0f);
  SlotId ext_id = fc.bind_external<PointCloud>("external_cloud", &external_cloud);
  assert(fc.get<PointCloud>(ext_id).n == 1);
  assert(fc.get<PointCloud>(ext_id).x[0] == 99.0f);

  // === Trigger Frame Reset 1 ===
  fc.reset_frame();

  // Ephemeral must be cleared
  assert(fc.get<PointCloud>(ephem_id).n == 0);

  // Persistent must survive untouched
  assert(fc.get<int>(persist_id) == 100);

  // External must remain untouched
  assert(fc.get<PointCloud>(ext_id).n == 1);

  // History ping-pong:
  // "scan.prev" must now hold Frame 0 data (10.0f, 20.0f, 30.0f)
  // "scan" must now be reset and empty!
  const auto& frame1_prev = fc.get<PointCloud>(hist_prev_id);
  const auto& frame1_curr = fc.get<PointCloud>(hist_id);
  assert(frame1_prev.n == 1);
  assert(frame1_prev.x[0] == 10.0f);
  assert(frame1_curr.n == 0);

  // Write Frame 1 data
  fc.get_mut<PointCloud>(hist_id).push_back(40.0f, 50.0f, 60.0f);

  // === Trigger Frame Reset 2 ===
  fc.reset_frame();

  // History ping-pong 2:
  // "scan.prev" must now hold Frame 1 data (40.0f, 50.0f, 60.0f)
  // "scan" must now be reset and ready for Frame 2!
  const auto& frame2_prev = fc.get<PointCloud>(hist_prev_id);
  const auto& frame2_curr = fc.get<PointCloud>(hist_id);
  assert(frame2_prev.n == 1);
  assert(frame2_prev.x[0] == 40.0f);
  assert(frame2_curr.n == 0);

  std::cout << "  ✓ SlotLifetime policies verified." << std::endl;
}

static void test_clone_prototype() {
  std::cout << "[Test 7] FrameContext clone_prototype zero-heap scratch replication..." << std::endl;
  FrameContext prototype;
  SlotId c1 = prototype.get_or_register_slot<PointCloud>("cloud");
  prototype.preallocate_buffers(5000);
  prototype.get_or_register_param<float>("threshold", 0.75f);

  PointCloud ext_cloud;
  ext_cloud.push_back(1.0f, 1.0f, 1.0f);
  prototype.bind_external<PointCloud>("ext", &ext_cloud);

  // Clone
  FrameContext worker_fc = prototype.clone_prototype();

  assert(worker_fc.size() == prototype.size());
  assert(worker_fc.find("cloud").is_valid());
  assert(worker_fc.find("ext").is_valid());

  // Check pre-reserved scratch capacity
  const auto& worker_cloud = worker_fc.get<PointCloud>(c1);
  assert(worker_cloud.n == 0);
  assert(worker_cloud.x.capacity() >= 5000); // Pre-reserved!

  // Check parameter copy
  assert(std::abs(worker_fc.get_param<float>(worker_fc.get_id("threshold")) - 0.75f) < 1e-6f);

  // Check external pointer replication
  assert(worker_fc.get<PointCloud>("ext").n == 1);
  assert(worker_fc.get<PointCloud>("ext").x[0] == 1.0f);

  std::cout << "  ✓ clone_prototype scratch replication verified." << std::endl;
}

static void test_cluster_topology() {
  std::cout << "[Test 8] ClusterTopology and core affinity..." << std::endl;
  ClusterTopology topo = ClusterTopology::spacemit_k1_default();

  assert(topo.num_clusters() == 2);
  const CoreCluster* c0 = topo.find_cluster("Cluster0");
  const CoreCluster* c1 = topo.find_cluster("Cluster1");
  assert(c0 != nullptr && c1 != nullptr);

  assert(c0->contains(0) && c0->contains(1) && c0->contains(2) && c0->contains(3));
  assert(!c0->contains(4));
  assert(c1->contains(4) && c1->contains(5) && c1->contains(6) && c1->contains(7));
  assert(!c1->contains(0));

#if defined(__linux__)
  bool ok = set_current_thread_affinity(*c0);
  (void)ok;
#endif

  std::cout << "  ✓ ClusterTopology construction and queries verified." << std::endl;
}

static void test_stream_resequencer_and_queue() {
  std::cout << "[Test 9] StreamResequencer and FrameQueue..." << std::endl;

  // 1. Resequencer out-of-order reassembly
  std::vector<uint64_t> emitted_seqs;
  std::vector<std::string> emitted_data;
  StreamResequencer<std::string> reseq([&](uint64_t seq, std::string val) {
    emitted_seqs.push_back(seq);
    emitted_data.push_back(val);
  });

  // Push out of order: 2, 0, 1, 4, 3
  reseq.push(2, "frame_2");
  assert(emitted_seqs.empty()); // 0 not received yet!
  assert(reseq.pending_count() == 1);

  reseq.push(0, "frame_0");
  assert(emitted_seqs.size() == 1 && emitted_seqs[0] == 0); // 0 emitted, waiting on 1

  reseq.push(1, "frame_1");
  assert(emitted_seqs.size() == 3); // 0, 1, 2 emitted!
  assert(emitted_seqs[1] == 1 && emitted_seqs[2] == 2);
  assert(reseq.pending_count() == 0);

  reseq.push(4, "frame_4");
  reseq.push(3, "frame_3");
  assert(emitted_seqs.size() == 5);
  for (uint64_t i = 0; i < 5; ++i) {
    assert(emitted_seqs[i] == i);
  }

  // 2. FrameQueue IngressPolicy::DropOldest
  FrameQueue<int> drop_queue(2, IngressPolicy::DropOldest);
  drop_queue.push(10);
  drop_queue.push(20);
  drop_queue.push(30); // drops 10
  assert(drop_queue.dropped_count() == 1);
  assert(drop_queue.size() == 2);

  int val = 0;
  assert(drop_queue.pop(val) && val == 20);
  assert(drop_queue.pop(val) && val == 30);

  std::cout << "  ✓ StreamResequencer and FrameQueue policies verified." << std::endl;
}

static void test_frame_worker_pool() {
  std::cout << "[Test 10] FrameWorkerPool streaming and resequencing..." << std::endl;

  PipelineManager pm;
  pm.set_execution_mode(ExecutionMode::FrameWorkerPool);
  assert(pm.execution_mode() == ExecutionMode::FrameWorkerPool);

  pm.add_node("passthrough_scale",
      in<PointCloud>("raw_in"),
      out<PointCloud>("scaled_out"),
      param<float>("scale", 2.0f)
  )
  .kernel([&](const PointCloud& src, PointCloud& dst, float s) {
    dst.clear();
    for (std::size_t i = 0; i < src.n; ++i) {
      dst.push_back(src.x[i] * s, src.y[i] * s, src.z[i] * s);
    }
  });

  pm.set_primary_input("raw_in");
  pm.initialize(512);

  // Generate 8 test frames
  std::vector<PointCloud> frames(8);
  for (size_t i = 0; i < 8; ++i) {
    frames[i].reserve(10);
    for (size_t p = 0; p < 5; ++p) {
      frames[i].push_back(static_cast<float>(i + 1), 0.0f, 0.0f);
    }
  }

  // Run through worker pool with 4 workers and resequencing
  std::vector<uint64_t> received_seqs;
  std::vector<float> received_first_x;
  RegisterId out_id = pm.get_id("scaled_out");

  pm.run_worker_pool_resequenced(
      frames,
      [out_id](const FrameContext& fc) -> float {
        const auto& out = fc.get<PointCloud>(out_id);
        return (out.n > 0) ? out.x[0] : 0.0f;
      },
      [&](uint64_t seq_id, float first_x) {
        received_seqs.push_back(seq_id);
        received_first_x.push_back(first_x);
      },
      4 // 4 workers
  );

  assert(received_seqs.size() == 8);
  for (size_t i = 0; i < 8; ++i) {
    assert(received_seqs[i] == i);
    // (i + 1) * 2.0 = first_x
    float expected_x = static_cast<float>(i + 1) * 2.0f;
    assert(std::abs(received_first_x[i] - expected_x) < 1e-5f);
  }

  std::cout << "  ✓ FrameWorkerPool multi-worker execution and resequencing verified." << std::endl;
}

int main() {
  std::cout << "=== Running Fast Tests: Production Pipeline Engine ===" << std::endl;

  test_register_file_basics();
  test_dag_topological_sort();
  test_cycle_detection();
  test_interleaved_tagged_binding();
  test_dynamic_reconfiguration_and_probes();
  test_slot_lifetime_policies();
  test_clone_prototype();
  test_cluster_topology();
  test_stream_resequencer_and_queue();
  test_frame_worker_pool();

  std::cout << "\n>>> ALL PIPELINE ENGINE TESTS PASSED! <<<" << std::endl;
  return 0;
}

