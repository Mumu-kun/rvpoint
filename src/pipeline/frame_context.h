#pragma once

#include "core/point_types.h"
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include <memory>
#include <typeindex>
#include <functional>
#include <stdexcept>
#include <cassert>
#include <type_traits>

namespace rvpoint {

// SFINAE Helpers for Generic Lifecycle Management
template <typename T, typename = void>
struct has_clear : std::false_type {};
template <typename T>
struct has_clear<T, std::void_t<decltype(std::declval<T&>().clear())>> : std::true_type {};
template <typename T>
inline constexpr bool has_clear_v = has_clear<T>::value;

template <typename T, typename = void>
struct has_reserve : std::false_type {};
template <typename T>
struct has_reserve<T, std::void_t<decltype(std::declval<T&>().reserve(std::declval<std::size_t>()))>> : std::true_type {};
template <typename T>
inline constexpr bool has_reserve_v = has_reserve<T>::value;

template <typename T, typename = void>
struct has_capacity : std::false_type {};
template <typename T>
struct has_capacity<T, std::void_t<decltype(std::declval<const T&>().capacity())>> : std::true_type {};
template <typename T>
inline constexpr bool has_capacity_v = has_capacity<T>::value;

/**
 * @brief Zero-overhead 16-bit handle for register slots and parameters.
 */
struct RegisterId {
    uint16_t index = 0xFFFF;
    constexpr bool is_valid() const noexcept { return index != 0xFFFF; }
    constexpr bool operator==(RegisterId o) const noexcept { return index == o.index; }
    constexpr bool operator!=(RegisterId o) const noexcept { return index != o.index; }
};

using SlotId = RegisterId;

using ParamValue = std::variant<float, double, int, uint32_t, size_t, bool, std::string>;

enum class RegisterKind { DataSlot, Parameter };

/**
 * @brief Slot lifecycle policy across consecutive frames (ADR-0013).
 */
enum class SlotLifetime {
    Ephemeral,   ///< Automatically cleared on reset_frame() while retaining capacity
    Persistent,  ///< Retained untouched across frame resets
    History,     ///< Ping-pong ring buffer rotating between [t] and [t-1]
    External     ///< Non-owning view of memory managed outside the pipeline
};

using SlotRetention = SlotLifetime; // Backwards compatibility alias

struct RegisterEntry {
    std::string name;
    RegisterKind kind = RegisterKind::DataSlot;
    SlotLifetime lifetime = SlotLifetime::Ephemeral;
    std::type_index type_id = typeid(void);
    void* instance = nullptr;
    std::unique_ptr<void, void(*)(void*)> storage{nullptr, [](void*){}};

    std::function<void(void*)> reset_fn;
    std::function<void(void*, std::size_t)> reserve_fn;
    std::function<RegisterEntry(const RegisterEntry&)> clone_scratch_fn;
    ParamValue param_val;
};

struct HistoryPair {
    RegisterId primary_id;
    RegisterId prev_id;
};

/**
 * @brief Typed execution context storing pre-allocated representations and parameters.
 *
 * Implements ADR-0012 and ADR-0013: Zero-copy pointer passing, steady-state zero-heap
 * allocations, four SlotLifetime policies, and thread-safe prototype cloning for worker pools.
 */
class FrameContext {
public:
    template <typename T, typename... Args>
    RegisterId get_or_register_slot(const std::string& name, SlotLifetime lifetime, Args&&... args) {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) {
            assert(entries_[it->second.index].type_id == typeid(T) && "Register slot type mismatch!");
            return it->second;
        }

        RegisterId id{static_cast<uint16_t>(entries_.size())};
        RegisterEntry entry;
        entry.name = name;
        entry.kind = RegisterKind::DataSlot;
        entry.lifetime = lifetime;
        entry.type_id = typeid(T);

        T* obj = new T(std::forward<Args>(args)...);
        entry.instance = obj;
        entry.storage = std::unique_ptr<void, void(*)(void*)>(obj, [](void* p) {
            delete static_cast<T*>(p);
        });

        if constexpr (has_clear_v<T>) {
            entry.reset_fn = [](void* p) { static_cast<T*>(p)->clear(); };
        }
        if constexpr (has_reserve_v<T>) {
            entry.reserve_fn = [](void* p, std::size_t cap) { static_cast<T*>(p)->reserve(cap); };
        }

        entry.clone_scratch_fn = [](const RegisterEntry& src) -> RegisterEntry {
            RegisterEntry clone;
            clone.name = src.name;
            clone.kind = src.kind;
            clone.lifetime = src.lifetime;
            clone.type_id = src.type_id;

            T* new_obj = new T();
            if constexpr (has_reserve_v<T> && has_capacity_v<T>) {
                const T* src_obj = static_cast<const T*>(src.instance);
                if (src_obj) {
                    new_obj->reserve(src_obj->capacity());
                }
            }
            clone.instance = new_obj;
            clone.storage = std::unique_ptr<void, void(*)(void*)>(new_obj, [](void* p) {
                delete static_cast<T*>(p);
            });
            if constexpr (has_clear_v<T>) {
                clone.reset_fn = [](void* p) { static_cast<T*>(p)->clear(); };
            }
            if constexpr (has_reserve_v<T>) {
                clone.reserve_fn = [](void* p, std::size_t cap) { static_cast<T*>(p)->reserve(cap); };
            }
            clone.clone_scratch_fn = src.clone_scratch_fn;
            return clone;
        };

        entries_.push_back(std::move(entry));
        name_to_id_[name] = id;
        return id;
    }

    template <typename T, typename... Args>
    RegisterId get_or_register_slot(const std::string& name, Args&&... args) {
        return get_or_register_slot<T>(name, SlotLifetime::Ephemeral, std::forward<Args>(args)...);
    }

    template <typename T, typename... Args>
    RegisterId get_or_register_history(const std::string& name, Args&&... args) {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) {
            return it->second;
        }

        // 1. Primary slot (current frame [t])
        RegisterId primary_id = get_or_register_slot<T>(name, SlotLifetime::History, std::forward<Args>(args)...);

        // 2. Historical slot (previous frame [t-1])
        std::string prev_name = name + ".prev";
        RegisterId prev_id = get_or_register_slot<T>(prev_name, SlotLifetime::History, std::forward<Args>(args)...);

        history_slots_.push_back({primary_id, prev_id});
        return primary_id;
    }

    template <typename T>
    RegisterId bind_external(const std::string& name, T* external_ptr) {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) {
            assert(entries_[it->second.index].type_id == typeid(T) && "External slot type mismatch!");
            entries_[it->second.index].instance = external_ptr;
            entries_[it->second.index].lifetime = SlotLifetime::External;
            return it->second;
        }

        RegisterId id{static_cast<uint16_t>(entries_.size())};
        RegisterEntry entry;
        entry.name = name;
        entry.kind = RegisterKind::DataSlot;
        entry.lifetime = SlotLifetime::External;
        entry.type_id = typeid(T);
        entry.instance = external_ptr;
        entry.storage = std::unique_ptr<void, void(*)(void*)>(nullptr, [](void*){});

        entry.clone_scratch_fn = [](const RegisterEntry& src) -> RegisterEntry {
            RegisterEntry clone;
            clone.name = src.name;
            clone.kind = src.kind;
            clone.lifetime = src.lifetime;
            clone.type_id = src.type_id;
            clone.instance = src.instance; // Replicate non-owning external pointer
            clone.storage = std::unique_ptr<void, void(*)(void*)>(nullptr, [](void*){});
            clone.clone_scratch_fn = src.clone_scratch_fn;
            return clone;
        };

        entries_.push_back(std::move(entry));
        name_to_id_[name] = id;
        return id;
    }

    template <typename T>
    RegisterId get_or_register_param(const std::string& name, T default_val) {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) return it->second;

        RegisterId id{static_cast<uint16_t>(entries_.size())};
        RegisterEntry entry;
        entry.name = name;
        entry.kind = RegisterKind::Parameter;
        entry.lifetime = SlotLifetime::Persistent;
        entry.type_id = typeid(T);
        entry.param_val = ParamValue(default_val);

        entry.clone_scratch_fn = [](const RegisterEntry& src) -> RegisterEntry {
            RegisterEntry clone;
            clone.name = src.name;
            clone.kind = src.kind;
            clone.lifetime = src.lifetime;
            clone.type_id = src.type_id;
            clone.param_val = src.param_val;
            clone.clone_scratch_fn = src.clone_scratch_fn;
            return clone;
        };

        entries_.push_back(std::move(entry));
        name_to_id_[name] = id;
        return id;
    }

    RegisterId find(const std::string& name) const noexcept {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) return it->second;
        return RegisterId{};
    }

    RegisterId get_id(const std::string& name) const {
        auto it = name_to_id_.find(name);
        if (it == name_to_id_.end()) {
            throw std::runtime_error("Register not found in FrameContext: " + name);
        }
        return it->second;
    }

    template <typename T>
    inline T& get_mut(RegisterId id) noexcept {
        if constexpr (std::is_same_v<T, ParamValue>) {
            return entries_[id.index].param_val;
        } else {
            return *reinterpret_cast<T*>(entries_[id.index].instance);
        }
    }

    template <typename T>
    inline const T& get(RegisterId id) const noexcept {
        if constexpr (std::is_same_v<T, ParamValue>) {
            return entries_[id.index].param_val;
        } else {
            return *reinterpret_cast<const T*>(entries_[id.index].instance);
        }
    }

    template <typename T>
    inline const T& get(const std::string& name) const {
        return get<T>(get_id(name));
    }

    template <typename T>
    inline T& get_mut(const std::string& name) {
        return get_mut<T>(get_id(name));
    }

    template <typename T>
    inline T get_param(RegisterId id) const {
        return std::get<T>(entries_[id.index].param_val);
    }

    template <typename T>
    void set_param(RegisterId id, T val) {
        ParamValue new_v(val);
        if (entries_[id.index].param_val != new_v) {
            entries_[id.index].param_val = new_v;
            dirty_ = true;
            changed_params_.push_back(id);
        }
    }

    template <typename T>
    void set_param(const std::string& name, T val) {
        set_param(get_id(name), val);
    }

    const void* get_pointer(RegisterId id) const noexcept {
        if (entries_[id.index].kind == RegisterKind::Parameter) {
            return &entries_[id.index].param_val;
        }
        return entries_[id.index].instance;
    }

    std::type_index type_id(RegisterId id) const noexcept {
        return entries_[id.index].type_id;
    }

    void reset_frame() {
        for (auto& entry : entries_) {
            if (entry.kind == RegisterKind::DataSlot) {
                if (entry.lifetime == SlotLifetime::Ephemeral && entry.reset_fn) {
                    entry.reset_fn(entry.instance);
                }
                // Persistent and External: untouched
            }
        }

        // Ping-pong history slots
        for (auto& pair : history_slots_) {
            RegisterEntry& curr_entry = entries_[pair.primary_id.index];
            RegisterEntry& prev_entry = entries_[pair.prev_id.index];

            void* old_curr = curr_entry.instance;
            void* old_prev = prev_entry.instance;

            // Swap pointers in O(1)
            curr_entry.instance = old_prev;
            prev_entry.instance = old_curr;

            // Reset the newly active current buffer for the upcoming frame
            if (curr_entry.reset_fn) {
                curr_entry.reset_fn(curr_entry.instance);
            }
        }
    }

    void preallocate_buffers(std::size_t capacity) {
        for (auto& entry : entries_) {
            if (entry.kind == RegisterKind::DataSlot && entry.reserve_fn) {
                entry.reserve_fn(entry.instance, capacity);
            }
        }
    }

    FrameContext clone_prototype() const {
        FrameContext clone;
        clone.entries_.reserve(entries_.size());
        for (const auto& entry : entries_) {
            if (entry.clone_scratch_fn) {
                clone.entries_.push_back(entry.clone_scratch_fn(entry));
            } else {
                RegisterEntry copy;
                copy.name = entry.name;
                copy.kind = entry.kind;
                copy.lifetime = entry.lifetime;
                copy.type_id = entry.type_id;
                copy.instance = entry.instance;
                copy.param_val = entry.param_val;
                clone.entries_.push_back(std::move(copy));
            }
        }
        clone.name_to_id_ = name_to_id_;
        clone.history_slots_ = history_slots_;
        clone.dirty_ = false;
        return clone;
    }

    bool is_dirty() const noexcept { return dirty_; }
    void clear_dirty() noexcept { dirty_ = false; changed_params_.clear(); }
    const std::vector<RegisterId>& changed_params() const noexcept { return changed_params_; }

    std::size_t size() const noexcept { return entries_.size(); }
    const std::string& name(RegisterId id) const noexcept { return entries_[id.index].name; }
    RegisterKind kind(RegisterId id) const noexcept { return entries_[id.index].kind; }
    SlotLifetime lifetime(RegisterId id) const noexcept { return entries_[id.index].lifetime; }

private:
    std::vector<RegisterEntry> entries_;
    std::unordered_map<std::string, RegisterId> name_to_id_;
    std::vector<HistoryPair> history_slots_;
    bool dirty_ = false;
    std::vector<RegisterId> changed_params_;
};

using RegisterFile = FrameContext;
using ConfigStore = FrameContext;
using PipelineContext = FrameContext;

} // namespace rvpoint
