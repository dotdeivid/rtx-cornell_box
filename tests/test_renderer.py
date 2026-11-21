"""Tests para Ray, RenderConfig, path_tracer, Renderer, load_obj y CLI de main.py."""

from pathlib import Path
import numpy as np
import pytest

from main import PRESETS, apply_preset, build_configs_from_args, parse_args
from src.camera import Camera
from src.config import CameraConfig, RenderConfig
from src.geometry import BVHNode, Quad, Sphere, Triangle
from src.materials import DiffuseMaterial, EmissiveMaterial
from src.ray import Ray
from src.renderer import Renderer, calculate_nee, color_ray
from src.scene import SceneMode, create_cornell_box_scene
from src.utils import load_obj, random_in_unit_disk, random_in_unit_sphere
from src.vector import Vec3


class TestRay:
    """Tests para Ray."""

    def test_direction_is_normalized(self):
        ray = Ray(Vec3(0, 0, 0), Vec3(3, 4, 0))
        assert abs(ray.direction.length() - 1.0) < 1e-6

    def test_point_at(self):
        ray = Ray(Vec3(1, 2, 3), Vec3(0, 0, 5))
        assert ray.point_at(4.0) == Vec3(1, 2, 7)


class TestRenderConfig:
    """Tests para RenderConfig."""

    def test_defaults_and_aspect_ratio(self):
        cfg = RenderConfig(width=800, height=400)
        assert cfg.aspect_ratio == 2.0
        assert isinstance(cfg.output_path, Path)

    def test_string_output_path_converted(self):
        cfg = RenderConfig(output_path="output/test.png")  # type: ignore[arg-type]
        assert isinstance(cfg.output_path, Path)

    def test_validations(self):
        with pytest.raises(ValueError):
            RenderConfig(width=0)
        with pytest.raises(ValueError):
            RenderConfig(height=-1)
        with pytest.raises(ValueError):
            RenderConfig(samples=0)
        with pytest.raises(ValueError):
            RenderConfig(max_depth=-1)
        with pytest.raises(ValueError):
            RenderConfig(gamma=0)
        with pytest.raises(ValueError):
            RenderConfig(max_sample=0.0)


class TestUtils:
    """Tests para funciones utilitarias."""

    def test_random_samplers_within_bounds(self):
        for _ in range(50):
            s = random_in_unit_sphere()
            assert s.length() < 1.0
            d = random_in_unit_disk()
            assert d.z == 0.0
            assert d.length() < 1.0

    def test_load_obj_valid_and_missing(self, tmp_path: Path):
        obj_file = tmp_path / "quad.obj"
        obj_file.write_text(
            "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n",
            encoding="utf-8",
        )
        tris = load_obj(obj_file, scale=2.0, offset=Vec3(1, 0, 0))
        # Un quad se triangula en 2 triángulos (fan)
        assert len(tris) == 2
        assert tris[0].v0 == Vec3(1, 0, 0)
        assert tris[0].v1 == Vec3(3, 0, 0)

        with pytest.raises(OSError):
            load_obj(tmp_path / "does_not_exist.obj")

    def test_load_obj_ground_y_aligns_lowest_vertex(self, tmp_path: Path):
        obj_file = tmp_path / "tri.obj"
        obj_file.write_text("v 0 5 0\nv 1 7 0\nv 0 6 1\nf 1 2 3\n", encoding="utf-8")
        tris = load_obj(obj_file, scale=2.0, ground_y=0.0)
        assert min(t.v0.y for t in tris) == 0.0
        assert tris[0].v1.y == 4.0

    def test_load_obj_invalid_content_raises(self, tmp_path: Path):
        no_faces = tmp_path / "empty.obj"
        no_faces.write_text("v 0 0 0\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_obj(no_faces)

        bad_index = tmp_path / "bad.obj"
        bad_index.write_text("v 0 0 0\nv 1 0 0\nf 1 2 9\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_obj(bad_index)

        bad_neg_index = tmp_path / "bad_neg.obj"
        bad_neg_index.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf -4 2 3\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_obj(bad_neg_index)

        short_face = tmp_path / "short_face.obj"
        short_face.write_text("v 0 0 0\nv 1 0 0\nf 1 2\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_obj(short_face)

        no_vertices = tmp_path / "no_verts.obj"
        no_vertices.write_text("f 1 2 3\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_obj(no_vertices)


class TestPathTracerAndRenderer:
    """Tests de integración para path_tracer y Renderer."""

    def test_color_ray_zero_depth_returns_black(self):
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)
        ray = Ray(Vec3(278, 278, -800), Vec3(0, 0, 1))
        assert color_ray(ray, world, lights, depth=0, max_depth=4) == Vec3(0, 0, 0)

    def test_color_ray_hits_light_directly(self):
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)
        # Rayo apuntando directo al centro de la luz del techo (y=554)
        ray = Ray(Vec3(278, 300, 278), Vec3(0, 1, 0))
        radiance = color_ray(ray, world, lights, depth=4, max_depth=4, can_see_light=True)
        assert radiance == Vec3(15, 15, 15)

        # Con can_see_light=False no suma emisión directa (evita doble conteo con NEE)
        radiance_hidden = color_ray(
            ray, world, lights, depth=4, max_depth=4, can_see_light=False
        )
        assert radiance_hidden == Vec3(0, 0, 0)

    def test_unsampled_emitter_still_contributes_when_can_see_light_false(self):
        """Un emisor que NO está en lights no debe silenciarse tras rebote difuso."""
        emissive = EmissiveMaterial(Vec3(7, 7, 7))
        # Normal hacia -z (frente al rayo que parte del origen hacia +z)
        tri_light = Triangle(Vec3(-1, -1, 5), Vec3(0, 1, 5), Vec3(1, -1, 5), emissive)
        ray = Ray(Vec3(0, 0, 0), Vec3(0, 0, 1))
        # lights=[] -> no muestreado por NEE, debe emitir incluso con can_see_light=False
        radiance = color_ray(ray, tri_light, [], depth=4, max_depth=4, can_see_light=False)
        assert radiance == Vec3(7, 7, 7)

    def test_emissive_sphere_supports_nee(self):
        """Una esfera emisiva implementa AreaLight y aporta iluminación directa por NEE."""
        light_sphere = Sphere(Vec3(0, 10, 0), 2.0, EmissiveMaterial(Vec3(10, 10, 10)))
        floor_mat = DiffuseMaterial(Vec3(0.8, 0.8, 0.8))
        floor = Quad(Vec3(-5, 0, -5), Vec3(0, 0, 10), Vec3(10, 0, 0), floor_mat)
        world = BVHNode.create([light_sphere, floor])

        hit = floor.hit(Ray(Vec3(0, 5, 0), Vec3(0, -1, 0)), 0.001, 100.0)
        assert hit is not None
        direct = calculate_nee(hit, world, [light_sphere], floor_mat.albedo)
        assert direct.x > 0.0 and direct.y > 0.0 and direct.z > 0.0

    def test_consecutive_seeds_do_not_share_shifted_rows(self, tmp_path: Path):
        """Semillas consecutivas no deben producir filas idénticas desplazadas."""
        world, lights = create_cornell_box_scene(SceneMode.SPHERES)
        camera = Camera(CameraConfig(aperture=0.0), aspect_ratio=1.0)
        cfg1 = RenderConfig(
            width=10, height=4, samples=2, max_depth=3,
            use_parallel=False, seed=1, output_path=tmp_path / "s1.png",
        )
        cfg2 = RenderConfig(
            width=10, height=4, samples=2, max_depth=3,
            use_parallel=False, seed=2, output_path=tmp_path / "s2.png",
        )
        row_s1_y1 = Renderer(cfg1)._render_row(1, world, lights, camera)
        row_s2_y0 = Renderer(cfg2)._render_row(0, world, lights, camera)
        assert not np.array_equal(row_s1_y1, row_s2_y0)

    def test_calculate_nee_illuminates_floor(self):
        light_mat = EmissiveMaterial(Vec3(10, 10, 10))
        floor_mat = DiffuseMaterial(Vec3(0.8, 0.8, 0.8))
        # Luz en y=10 con normal -y
        light = Quad(Vec3(0, 10, 0), Vec3(2, 0, 0), Vec3(0, 0, 2), light_mat)
        floor = Quad(Vec3(-5, 0, -5), Vec3(0, 0, 10), Vec3(10, 0, 0), floor_mat)
        world = BVHNode.create([light, floor])

        hit = floor.hit(Ray(Vec3(1, 5, 1), Vec3(0, -1, 0)), 0.001, 100.0)
        assert hit is not None
        direct = calculate_nee(hit, world, [light], floor_mat.albedo)
        assert direct.x > 0.0 and direct.y > 0.0 and direct.z > 0.0

    def test_renderer_sequential_and_parallel_deterministic_with_seed(self, tmp_path: Path):
        world, lights = create_cornell_box_scene(SceneMode.BOXES)
        camera = Camera(CameraConfig(aperture=0.0), aspect_ratio=1.0)

        out_seq = tmp_path / "seq.png"
        out_par = tmp_path / "par.png"

        cfg_seq = RenderConfig(
            width=12,
            height=12,
            samples=4,
            max_depth=4,
            use_parallel=False,
            seed=42,
            output_path=out_seq,
        )
        cfg_par = RenderConfig(
            width=12,
            height=12,
            samples=4,
            max_depth=4,
            use_parallel=True,
            seed=42,
            output_path=out_par,
        )

        img_seq = Renderer(cfg_seq).render(world, lights, camera)
        img_par = Renderer(cfg_par).render(world, lights, camera)

        assert out_seq.exists()
        assert out_par.exists()
        assert img_seq.shape == (12, 12, 3)
        assert np.array_equal(img_seq, img_par)
        # La imagen no debe ser completamente negra
        assert int(img_seq.sum()) > 0


class TestMainCLI:
    """Tests para presets y CLI en main.py."""

    def test_apply_preset_valid_and_invalid(self):
        rcfg, ccfg, mode = apply_preset("preview")
        assert rcfg.width == PRESETS["PREVIEW"].width
        assert ccfg.aperture == 0.0
        assert mode == SceneMode.SPHERES

        with pytest.raises(KeyError):
            apply_preset("NO_EXISTE")

    def test_parse_args_rejects_unknown_preset(self):
        assert parse_args(["--preset", "preview"]).preset == "PREVIEW"
        with pytest.raises(SystemExit):
            parse_args(["--preset", "NO_EXISTE"])

    def test_cli_overrides(self):
        args = parse_args(
            [
                "--preset",
                "PREVIEW",
                "--scene",
                "boxes",
                "--width",
                "64",
                "--height",
                "48",
                "--samples",
                "16",
                "--max-depth",
                "6",
                "--aperture",
                "5.0",
                "--seed",
                "123",
                "--sequential",
                "--output",
                "output/custom.png",
            ]
        )
        rcfg, ccfg, mode = build_configs_from_args(args)
        assert mode == SceneMode.BOXES
        assert rcfg.width == 64
        assert rcfg.height == 48
        assert rcfg.samples == 16
        assert rcfg.max_depth == 6
        assert rcfg.use_parallel is False
        assert rcfg.seed == 123
        assert rcfg.output_path == Path("output/custom.png")
        assert ccfg.aperture == 5.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
