#pragma once

#include "core/point_types.h"
#include "pipeline/register_file.h"
#include "pipeline/tagged_binding.h"

#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include <chrono>
#include <memory>
#include <type_traits>
#include <cassert>
#include <stdexcept>
#include <iostream>

namespace rvpoint {

/**
 * @brief Hook attached to a specific parameter ID that fires whenever the parameter changes.
 */
struct ReconfigHook {
  RegisterId param_id;
  std::function<void(RegisterFile&, const ParamValue&)> callback;
};

/**
 * @brief Positional binding record maintaining the exact sequence of arguments passed to add_node.
 */
struct ArgBinding {
  RegisterId id;
  TagKind kind;
};

/**
 * @brief A compiled pipeline stage node ready for zero-overhead execution.
 */
struct CompiledNode {
  std::string name;
  std::vector<RegisterId> input_slots;
  std::vector<RegisterId> output_slots;
  std::vector<RegisterId> param_ids;
  std::vector<ArgBinding> arg_bindings;

  // Pre-bound pointers for each argument in positional order (ZERO runtime hash lookups)
  std::vector<const void*> bound_args;

  // Direct thunk dispatch
  std::function<void(CompiledNode*)> invoke_thunk;

  // Reconfiguration hooks
  std::vector<ReconfigHook> reconfig_hooks;

  // Node completion callbacks
  std::vector<std::function<void(const CompiledNode&, double)>> completion_hooks;

  double last_execution_time_ms = 0.0;

  /**
   * @brief Type-safe argument fetcher used inside variadic invoker thunks.
   */
  template <typename ArgType, bool IsParam>
  inline ArgType get_arg(std::size_t idx) const noexcept {
    using BareType = std::remove_cv_t<std::remove_reference_t<ArgType>>;
    if constexpr (IsParam) {
      const auto* pv = reinterpret_cast<const ParamValue*>(bound_args[idx]);
      return std::get<BareType>(*pv);
    } else {
      if constexpr (std::is_reference_v<ArgType>) {
        if constexpr (std::is_const_v<std::remove_reference_t<ArgType>>) {
          return *reinterpret_cast<const BareType*>(bound_args[idx]);
        } else {
          return *const_cast<BareType*>(reinterpret_cast<const BareType*>(bound_args[idx]));
        }
      } else {
        return *reinterpret_cast<const BareType*>(bound_args[idx]);
      }
    }
  }
};

/**
 * @brief Declarative inspection probe attached to a slot.
 */
struct ProbeEntry {
  std::string slot_name;
  RegisterId slot_id;
  std::function<void(const RegisterFile&, RegisterId)> callback;
};

/**
 * @brief Central pipeline orchestration manager implementing ADR-0012.
 *
 * Provides Kahn's DAG topological ordering, compile-time tagged positional binding,
 * pre-bound pointer invocations (zero runtime lookups), zero-heap steady-state buffers,
 * and live parameter reconfiguration.
 */
class PipelineManager {
public:
  PipelineManager() = default;

  RegisterFile& registers() noexcept { return rf_; }
  const RegisterFile& registers() const noexcept { return rf_; }

  template <typename T>
  void set_param(const std::string& name, T val) {
    rf_.set_param(name, val);
  }

  RegisterId get_id(const std::string& name) const {
    return rf_.get_id(name);
  }

  void set_primary_input(std::string slot_name) {
    primary_input_name_ = std::move(slot_name);
  }

  // --- Node Builder ---
  template <typename... Tags>
  class NodeBuilder {
  public:
    NodeBuilder(PipelineManager& pm, std::string name, Tags... tags)
        : pm_(pm), name_(std::move(name)), tags_(tags...) {}

    template <typename T, typename HookFn>
    NodeBuilder& on_reconfig(const std::string& param_name, HookFn&& hook) {
      RegisterId pid = pm_.registers().template get_or_register_param<T>(param_name, T{});
      reconfig_hooks_.push_back({pid, [hook = std::forward<HookFn>(hook)](RegisterFile& rf, const ParamValue& val) {
        hook(rf, std::get<T>(val));
      }});
      return *this;
    }

    template <typename HookFn>
    NodeBuilder& on_completed(HookFn&& hook) {
      completion_hooks_.push_back(std::forward<HookFn>(hook));
      return *this;
    }

    template <typename KernelFn>
    void kernel(KernelFn&& fn) {
      pm_.register_node_variadic(
          name_, tags_, std::forward<KernelFn>(fn),
          std::move(reconfig_hooks_), std::move(completion_hooks_));
    }

  private:
    PipelineManager& pm_;
    std::string name_;
    std::tuple<Tags...> tags_;
    std::vector<ReconfigHook> reconfig_hooks_;
    std::vector<std::function<void(const CompiledNode&, double)>> completion_hooks_;
  };

  template <typename... Tags>
  NodeBuilder<Tags...> add_node(std::string name, Tags... tags) {
    return NodeBuilder<Tags...>(*this, std::move(name), tags...);
  }

  // --- Declarative Probes ---
  template <typename T, typename ProbeFn>
  void add_probe(std::string slot_name, ProbeFn&& probe) {
    ProbeEntry pe;
    pe.slot_name = std::move(slot_name);
    pe.callback = [probe = std::forward<ProbeFn>(probe)](const RegisterFile& rf, RegisterId id) {
      probe(rf.get<T>(id));
    };
    probes_.push_back(std::move(pe));
  }

  // --- Lifecycle Hooks ---
  void on_frame_start(std::function<void(RegisterFile&)> hook) {
    frame_start_hooks_.push_back(std::move(hook));
  }

  void on_frame_end(std::function<void(const RegisterFile&)> hook) {
    frame_end_hooks_.push_back(std::move(hook));
  }

  void initialize(std::size_t default_capacity = 131072);

  // --- Multi-Mode step() Execution Interface ---

  // Mode 1a: PointCloudView input streaming
  void step(const PointCloudView& cloud);

  // Mode 1b: PointCloud owning input streaming
  void step(const PointCloud& cloud) {
    step(cloud.view());
  }

  // Mode 2: Multi-sensor / custom feeder lambda
  template <typename FeederFn, typename = std::enable_if_t<std::is_invocable_v<FeederFn, RegisterFile&>>>
  void step(FeederFn&& feeder) {
    execute_frame_internal(std::forward<FeederFn>(feeder));
  }

  // Mode 3: Parameterless step (for autonomous daemons / pre-populated registers)
  void step() {
    execute_frame_internal([](RegisterFile&) {});
  }

  void print_telemetry() const;
  double last_pipeline_time_ms() const noexcept { return total_pipeline_time_ms_; }
  const std::vector<CompiledNode>& nodes() const noexcept { return nodes_; }

private:
  RegisterFile rf_;
  std::vector<CompiledNode> nodes_;
  std::vector<ProbeEntry> probes_;
  std::vector<std::function<void(RegisterFile&)>> frame_start_hooks_;
  std::vector<std::function<void(const RegisterFile&)>> frame_end_hooks_;

  std::string primary_input_name_;
  RegisterId primary_input_id_;
  double total_pipeline_time_ms_ = 0.0;

  template <typename FeederFn>
  void execute_frame_internal(FeederFn&& feeder);

  void resolve_dag_order();

  template <typename KernelFn, typename... Tags, std::size_t... Is>
  static void build_invoker(CompiledNode& node, KernelFn&& kernel, std::index_sequence<Is...>) {
    using DecayK = std::decay_t<KernelFn>;
    static_assert(std::is_invocable_v<DecayK&, typename std::tuple_element_t<Is, std::tuple<Tags...>>::ref_type...>,
                  "Kernel function signature does not match bound tags!");

    node.invoke_thunk = [k = std::forward<KernelFn>(kernel)](CompiledNode* self) mutable {
      k(self->template get_arg<
          typename std::tuple_element_t<Is, std::tuple<Tags...>>::ref_type,
          (std::tuple_element_t<Is, std::tuple<Tags...>>::kind == TagKind::Parameter)
        >(Is)...);
    };
  }

  template <typename Tuple, typename KernelFn>
  void register_node_variadic(
      const std::string& name, const Tuple& tags, KernelFn&& fn,
      std::vector<ReconfigHook> reconfig_hooks,
      std::vector<std::function<void(const CompiledNode&, double)>> completion_hooks)
  {
    CompiledNode node;
    node.name = name;
    node.reconfig_hooks = std::move(reconfig_hooks);
    node.completion_hooks = std::move(completion_hooks);

    std::apply([&](auto&&... tag) {
      (this->process_tag(node, tag), ...);
    }, tags);

    using IndexSeq = std::make_index_sequence<std::tuple_size_v<Tuple>>;
    build_invoker_helper(node, std::forward<KernelFn>(fn), tags, IndexSeq{});

    nodes_.push_back(std::move(node));
  }

  template <typename KernelFn, typename... Tags, std::size_t... Is>
  void build_invoker_helper(
      CompiledNode& node, KernelFn&& fn,
      const std::tuple<Tags...>&, std::index_sequence<Is...> seq)
  {
    build_invoker<KernelFn, Tags...>(node, std::forward<KernelFn>(fn), seq);
  }

  template <typename T>
  void process_tag(CompiledNode& node, const InTag<T>& tag) {
    RegisterId sid = rf_.template get_or_register_slot<T>(tag.name);
    node.input_slots.push_back(sid);
    node.arg_bindings.push_back({sid, TagKind::Input});
  }

  template <typename T>
  void process_tag(CompiledNode& node, const OutTag<T>& tag) {
    RegisterId sid = rf_.template get_or_register_slot<T>(tag.name);
    node.output_slots.push_back(sid);
    node.arg_bindings.push_back({sid, TagKind::Output});
  }

  template <typename T>
  void process_tag(CompiledNode& node, const ParamTag<T>& tag) {
    RegisterId pid = rf_.template get_or_register_param<T>(tag.name, tag.default_value);
    node.param_ids.push_back(pid);
    node.arg_bindings.push_back({pid, TagKind::Parameter});
  }
};

template <typename FeederFn>
void PipelineManager::execute_frame_internal(FeederFn&& feeder) {
  using Clock = std::chrono::high_resolution_clock;

  // Step 1: Logical frame reset (reuses pre-allocated capacity)
  rf_.reset_frame();

  // Step 2: Feed sensory inputs into register file
  feeder(rf_);

  // Step 3: Lifecycle frame start hooks
  for (auto& hook : frame_start_hooks_) {
    hook(rf_);
  }

  // Step 4: Dynamic parameter reconfiguration check
  if (__builtin_expect(rf_.is_dirty(), 0)) {
    for (auto pid : rf_.changed_params()) {
      const auto& new_val = rf_.get<ParamValue>(pid);
      for (auto& node : nodes_) {
        for (const auto& rh : node.reconfig_hooks) {
          if (rh.param_id == pid) {
            rh.callback(rf_, new_val);
          }
        }
      }
    }
    rf_.clear_dirty();
  }

  // Step 5: Sequential execution of DAG-ordered compiled nodes
  total_pipeline_time_ms_ = 0.0;
  for (auto& node : nodes_) {
    auto t0 = Clock::now();
    node.invoke_thunk(&node);
    auto t1 = Clock::now();
    node.last_execution_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_pipeline_time_ms_ += node.last_execution_time_ms;

    for (const auto& ch : node.completion_hooks) {
      ch(node, node.last_execution_time_ms);
    }
  }

  // Step 6: Execute declarative probes
  for (const auto& pe : probes_) {
    pe.callback(rf_, pe.slot_id);
  }

  // Step 7: Lifecycle frame end hooks
  for (auto& hook : frame_end_hooks_) {
    hook(rf_);
  }
}

} // namespace rvpoint

