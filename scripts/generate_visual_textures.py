#!/usr/bin/env python3
"""Generate deterministic, local visual textures for the Gazebo world."""

from pathlib import Path
import random

from PIL import Image, ImageDraw


SIZE = 2048
SEED = 0x57E2E0


def main():
    rng = random.Random(SEED)
    noise_size = 256
    pixels = bytes(
        142 + rng.randrange(-13, 14) for _ in range(noise_size * noise_size)
    )

    # Smooth, non-periodic aggregate variation prevents large uniform patches,
    # while avoiding pixel-scale noise and aliasing in the simulated cameras.
    image = Image.frombytes("L", (noise_size, noise_size), pixels).resize(
        (SIZE, SIZE), Image.Resampling.BICUBIC
    )
    draw = ImageDraw.Draw(image)

    # Multi-scale, uniquely placed glyphs provide stable corners from 0.15 m
    # out to the camera's 8 m clipping distance. No tile is repeated.
    for _ in range(1800):
        x = rng.randrange(18, SIZE - 18)
        y = rng.randrange(18, SIZE - 18)
        radius = rng.randrange(5, 30)
        tone = rng.choice((12, 22, 238, 248))
        shape = rng.randrange(5)
        width = rng.randrange(2, 7)
        if shape == 0:
            draw.line(
                (x - radius, y, x + radius, y + rng.randrange(-radius, radius + 1)),
                fill=tone,
                width=width,
            )
            draw.line((x, y - radius, x, y + radius // 2), fill=tone, width=width)
        elif shape == 1:
            draw.rectangle(
                (x - radius, y - radius // 2, x + radius, y + radius // 2),
                outline=tone,
                width=width,
            )
            draw.line((x - radius, y + radius // 2, x, y - radius // 2), fill=tone, width=width)
        elif shape == 2:
            draw.polygon(
                (
                    (x - radius, y + radius // 2),
                    (x - radius // 3, y - radius),
                    (x + radius, y + radius // 3),
                ),
                outline=tone,
            )
            draw.line((x - radius // 3, y - radius, x, y + radius // 2), fill=tone, width=width)
        elif shape == 3:
            draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                outline=tone,
                width=width,
            )
            draw.line((x - radius, y, x + radius // 2, y - radius // 2), fill=tone, width=width)
        else:
            draw.line((x - radius, y - radius, x + radius, y + radius), fill=tone, width=width)
            draw.line((x - radius, y + radius // 3, x + radius // 3, y - radius), fill=tone, width=width)

    output = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "stereo_nav_gazebo"
        / "materials"
        / "textures"
        / "indoor_floor.png"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, format="PNG", optimize=True)
    print(f"generated {output} ({output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
