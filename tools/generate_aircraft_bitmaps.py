from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from reportlab.graphics import renderPM
from svglib.svglib import svg2rlg


ROOT = Path(__file__).resolve().parents[1]
ICON_DIR = ROOT / "third_party" / "adsb-radar-icons" / "ADS-B_Radar_Free_Aircraft_SVG_Icons"
HEADER = ROOT / "arduino-radar" / "aircraft_bitmaps.h"
PREVIEW = ROOT / "docs" / "aircraft-bitmap-preview.png"

W = 64
H = 96
RENDER = 512

# Keep these as the original ADS-B Radar top-down silhouettes.
ART_SOURCES = {
    "HELICOPTER": "a7.svg",
    "LIGHT_PROP": "cessna.svg",
    "TWIN_PROP": "dh8a.svg",
    "BUSINESS_JET": "learjet.svg",
    "AIRLINER": "a320.svg",
    "HEAVY_JET": "b747.svg",
    "CARGO": "md11.svg",
    "MILITARY": "a6.svg",
    "GLIDER": "b1.svg",
    "BALLOON": "f11.svg",
    "UNKNOWN": "a0.svg",
}


def render_svg(svg_name):
    drawing = svg2rlg(str(ICON_DIR / svg_name))
    img = renderPM.drawToPIL(drawing, dpi=72).convert("L")
    img.thumbnail((RENDER, RENDER), Image.Resampling.LANCZOS)

    square = Image.new("L", (RENDER, RENDER), 255)
    square.paste(img, ((RENDER - img.width) // 2, (RENDER - img.height) // 2))
    return square.point(lambda p: 255 if p < 160 else 0)


def fit_to_bitmap(img):
    bbox = img.getbbox()
    if bbox:
        img = img.crop(bbox)
    img.thumbnail((W - 2, H - 2), Image.Resampling.LANCZOS)

    canvas = Image.new("L", (W, H), 0)
    canvas.paste(img, ((W - img.width) // 2, (H - img.height) // 2))
    return canvas.point(lambda p: 255 if p >= 92 else 0)


def bitmap_bytes(img):
    pixels = img.load()
    data = []
    bytes_per_row = (W + 7) // 8
    for y in range(H):
        for byte_x in range(bytes_per_row):
            value = 0
            for bit in range(8):
                x = byte_x * 8 + bit
                if x < W and pixels[x, y]:
                    value |= 0x80 >> bit
            data.append(value)
    return data


def write_header(bitmaps):
    out = []
    out.append("// Generated from tools/generate_aircraft_bitmaps.py.\n")
    out.append("// Source silhouettes: ADS-B Radar SVG icon set, https://adsb-radar.com/help/icons.html\n")
    out.append("// Bitmap bytes are row-packed for Adafruit_GFX drawBitmap().\n\n")
    out.append(f"constexpr int AIRCRAFT_BMP_W = {W};\n")
    out.append(f"constexpr int AIRCRAFT_BMP_H = {H};\n\n")
    for name, img in bitmaps.items():
        data = bitmap_bytes(img)
        out.append(f"const uint8_t AIRCRAFT_{name}_BMP[] PROGMEM = {{\n")
        for i in range(0, len(data), 14):
            out.append("  " + ", ".join(f"0x{b:02X}" for b in data[i:i + 14]))
            out.append(",\n" if i + 14 < len(data) else "\n")
        out.append("};\n\n")
    HEADER.write_text("".join(out).rstrip() + "\n", encoding="ascii")


def write_preview(bitmaps):
    scale = 2
    row_h = H * scale + 24
    preview = Image.new("RGB", (360, row_h * len(bitmaps)), "white")
    font = ImageFont.load_default()
    draw = ImageDraw.Draw(preview)
    for row, (name, img) in enumerate(bitmaps.items()):
        y = row * row_h
        scaled = img.convert("RGB").resize((W * scale, H * scale), Image.Resampling.NEAREST)
        preview.paste(scaled, (8, y + 6))
        draw.text((W * scale + 24, y + 12), f"{name} ({ART_SOURCES[name]})", fill="black", font=font)
    PREVIEW.parent.mkdir(parents=True, exist_ok=True)
    preview.save(PREVIEW)


def main():
    bitmaps = {name: fit_to_bitmap(render_svg(svg_name)) for name, svg_name in ART_SOURCES.items()}
    write_header(bitmaps)
    write_preview(bitmaps)
    print(f"Wrote {HEADER}")
    print(f"Wrote {PREVIEW}")


if __name__ == "__main__":
    main()
