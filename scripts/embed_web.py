import os, gzip
Import("env")
project_dir = env.get("PROJECT_DIR")

html_path = os.path.join(project_dir, "data", "index.html")
assets_path = os.path.join(project_dir, "include", "web_assets.h")

if not os.path.exists(assets_path) or os.path.getmtime(html_path) > os.path.getmtime(assets_path):

    # открываем файл html
    with open(html_path, "rb") as f:
        html_bytes = f.read()

    # сжимаем его с помощью gzip
    html_compressed = gzip.compress(html_bytes)

    # записываем в hex формате в строку
    hex_bytes = []
    for byte in html_compressed:
        hex_bytes.append(f"0x{byte:02x}")

    hex_string = ", ".join(hex_bytes)

    with open(assets_path, "w") as f:
        f.write(f"""#pragma once
#include <stdint.h>
#include <stddef.h>
#include <pgmspace.h>

const uint8_t web_index_gz[] PROGMEM = {{
{hex_string}
}};
const size_t web_index_gz_len = {len(html_compressed)};
""")
        print(">>> embed_web: web_assets.h regenerated")

else:
    print(">>> embed_web: web_assets.h is up to date, skipping")