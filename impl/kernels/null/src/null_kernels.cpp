// © Joseph Cameron - MIT License

#include <gdk/collisions/null_kernels.h>

#include <memory>

using namespace gdk::collisions;

namespace {
    class null_kernels final : public scene_kernels {
    public:
        virtual void statics_changed(const scene_tables &, std::span<const std::uint32_t>,
            std::span<const std::uint32_t>) override {}

        virtual void movers_changed(const scene_tables &, const delta_time_type) override {}

        virtual void gather_pairs(const scene_tables &, const pair_request &, std::vector<candidate_pair> &) override {}

        virtual void test_pairs(const scene_tables &, std::span<const pair_test>, std::vector<pair_hit> &) override {}

        [[nodiscard]] virtual std::optional<ray_answer> cast_ray(const scene_tables &, const ray_query &) const override {
            return std::nullopt;
        }

        [[nodiscard]] virtual bool meets_statics(const scene_tables &, const upright_cylinder &,
            std::span<const collider_id_type>) const override {
            return false;
        }

        [[nodiscard]] virtual bool meets_statics(const scene_tables &, const parallelepiped &,
            std::span<const collider_id_type>) const override {
            return false;
        }
    };
}

scene_kernels_ptr_type gdk::collisions::make_null_kernels() {
    return std::make_shared<null_kernels>();
}
