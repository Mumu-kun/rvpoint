#pragma once

#include <string>
#include <vector>
#include <cstdint>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace rvpoint {

/**
 * @brief Enforces the anti-oversubscription invariant (ADR-0013) across nested OpenMP runtimes.
 */
inline void enforce_anti_oversubscription() {
#if defined(_OPENMP)
    omp_set_max_active_levels(1);
#endif
}

/**
 * @brief A named hardware grouping of CPU cores sharing a unified cache hierarchy.
 *
 * Models physical silicon clusters such as the SpacemiT K1 dual 4-core clusters
 * (Cluster 0: Cores 0-3 with 1MB L2; Cluster 1: Cores 4-7 with 1MB L2).
 */
struct CoreCluster {
    std::string name;
    std::vector<int> core_ids;

    bool contains(int core_id) const noexcept {
        for (int id : core_ids) {
            if (id == core_id) return true;
        }
        return false;
    }
};

/**
 * @brief Hardware-aware cluster topology configuration.
 */
struct ClusterTopology {
    std::vector<CoreCluster> clusters;

    static ClusterTopology spacemit_k1_default() {
        return {
            {
                { "Cluster0", {0, 1, 2, 3} },
                { "Cluster1", {4, 5, 6, 7} }
            }
        };
    }

    const CoreCluster* find_cluster(const std::string& name) const noexcept {
        for (const auto& c : clusters) {
            if (c.name == name) return &c;
        }
        return nullptr;
    }

    std::size_t num_clusters() const noexcept {
        return clusters.size();
    }
};

/**
 * @brief Sets the CPU affinity of the calling thread to the specified CoreCluster under Linux.
 *
 * @param cluster The CoreCluster whose CPU IDs will be assigned to the thread.
 * @return true if affinity was successfully applied, false otherwise.
 */
inline bool set_current_thread_affinity(const CoreCluster& cluster) {
#if defined(__linux__)
    if (cluster.core_ids.empty()) return false;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int core_id : cluster.core_ids) {
        CPU_SET(core_id, &cpuset);
    }

    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    return (rc == 0);
#else
    (void)cluster;
    return false;
#endif
}

/**
 * @brief Convenience overload accepting a vector of core IDs.
 */
inline bool set_current_thread_affinity(const std::vector<int>& core_ids) {
    CoreCluster temp{"custom", core_ids};
    return set_current_thread_affinity(temp);
}

} // namespace rvpoint

