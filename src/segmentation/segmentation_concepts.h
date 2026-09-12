#pragma once

#include "core/point_types.h"
#include <type_traits>
#include <utility>

namespace rvpoint {

// ==============================================================================
// 1. Model Fitting Concept (RANSAC Plane, Sphere, Cylinder)
// ==============================================================================

namespace detail {

template <typename T, typename ModelT, typename = void>
struct is_model_fitter_impl : std::false_type {};

template <typename T, typename ModelT>
struct is_model_fitter_impl<
    T, ModelT,
    std::void_t<decltype(std::declval<T&>()(
        std::declval<const PointCloudView&>(),
        std::declval<ModelT&>()))>>
    : std::true_type {};

} // namespace detail

/**
 * @brief Trait checking whether T implements the ModelFitter concept for ModelT:
 *        int/bool operator()(const PointCloudView& in, ModelT& model)
 */
template <typename T, typename ModelT>
struct is_model_fitter : detail::is_model_fitter_impl<std::decay_t<T>, ModelT> {};

template <typename T, typename ModelT>
inline constexpr bool is_model_fitter_v = is_model_fitter<T, ModelT>::value;

/**
 * @brief Helper to fit a geometric model using any conforming ModelFitter.
 */
template <typename FitterT, typename ModelT>
inline auto fit_model(FitterT&& fitter, const PointCloudView& in, ModelT& model) {
    static_assert(is_model_fitter_v<FitterT, ModelT>,
                  "FitterT must satisfy is_model_fitter_v<FitterT, ModelT>");
    return fitter(in, model);
}

// ==============================================================================
// 2. Cluster Extractor Concept (Euclidean Clustering, Region Growing, DBSCAN)
// ==============================================================================

namespace detail {

template <typename T, typename = void>
struct is_cluster_extractor_impl : std::false_type {};

template <typename T>
struct is_cluster_extractor_impl<
    T,
    std::void_t<decltype(std::declval<T&>()(
        std::declval<const PointCloudView&>(),
        std::declval<ClusterResult&>()))>>
    : std::true_type {};

} // namespace detail

/**
 * @brief Trait checking whether T implements the ClusterExtractor concept:
 *        void operator()(const PointCloudView& in, ClusterResult& out)
 */
template <typename T>
struct is_cluster_extractor : detail::is_cluster_extractor_impl<std::decay_t<T>> {};

template <typename T>
inline constexpr bool is_cluster_extractor_v = is_cluster_extractor<T>::value;

/**
 * @brief Helper to extract topological clusters using any conforming ClusterExtractor.
 */
template <typename ClustererT>
inline void extract_clusters(ClustererT&& clusterer, const PointCloudView& in, ClusterResult& clusters) {
    static_assert(is_cluster_extractor_v<ClustererT>,
                  "ClustererT must satisfy is_cluster_extractor_v<ClustererT>");
    clusterer(in, clusters);
}

} // namespace rvpoint

