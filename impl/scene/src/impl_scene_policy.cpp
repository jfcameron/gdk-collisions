// © Joseph Cameron - MIT License

#include <gdk/collisions/exception.h>
#include <gdk/collisions/impl_scene_policy.h>

#include <cmath>
#include <string>

void gdk::collisions::require_valid_policy(const scene_policy &aPolicy) {
    const auto refuse = [](const char *aName, const char *aRule) {
        throw exception(std::string("gdk::collision: scene_policy::").append(aName).append(" must be ").append(aRule));
    };

    const auto at_least_one = [&](const auto aValue, const char *aName) {
        if (aValue < 1) refuse(aName, "at least 1");
    };

    const auto not_negative = [&](const floating_point_type aValue, const char *aName) {
        if (!std::isfinite(aValue) || aValue < 0) refuse(aName, "finite and not negative");
    };

    at_least_one(aPolicy.STEPS_PER_UPDATE, "STEPS_PER_UPDATE");
    at_least_one(aPolicy.CONSERVATIVE_ADVANCEMENT_MAX_STEPS, "CONSERVATIVE_ADVANCEMENT_MAX_STEPS");

    if (!(aPolicy.CLIP_PLANE_PARALLEL_COSINE > 0) || aPolicy.CLIP_PLANE_PARALLEL_COSINE > 1)
        refuse("CLIP_PLANE_PARALLEL_COSINE", "in (0, 1]");

    not_negative(aPolicy.TIME_OF_IMPACT_EPSILON, "TIME_OF_IMPACT_EPSILON");
    not_negative(aPolicy.AXIS_ENTRY_EPSILON, "AXIS_ENTRY_EPSILON");
    not_negative(aPolicy.MIN_SWEPT_SPEED, "MIN_SWEPT_SPEED");
    not_negative(aPolicy.PENETRATION_DEPTH_EPSILON, "PENETRATION_DEPTH_EPSILON");
    not_negative(aPolicy.NORMALIZATION_THRESHOLD, "NORMALIZATION_THRESHOLD");
}
