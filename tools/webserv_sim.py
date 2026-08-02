#!/usr/bin/env python3
"""Simulateur local de l'interface web du LEPIX_NODE (Core/Src/web_ui.c).

Rejoue en Python/HTTP standard le même rendu, la même navigation et les
mêmes endpoints CGI que le firmware, pour valider visuellement une page
sans flasher : mise en page, JS de la matrice de canaux, formulaires,
bandeau de test, hiérarchie par sortie. Ce n'est PAS le firmware — voir
la section « Fidélité » en bas de ce fichier pour ce qui diffère.

Lancement : make webserv   (ou directement : ./tools/webserv_sim.py)
Puis ouvrir http://127.0.0.1:8080/
"""
import argparse
import http.server
import json
import random
import socketserver
import threading
import time
from urllib.parse import urlparse, parse_qs, unquote

MAX_OUTPUTS = 4
DMX_SLOTS = 512
WS2815_MAX_LEDS = 512

PROTO_ARTNET, PROTO_SACN, PROTO_DMX_UART = 0, 1, 2
PROTO_NAMES = {PROTO_ARTNET: "Art-Net", PROTO_SACN: "sACN", PROTO_DMX_UART: "DMX (UART)"}

TEST_OFF, TEST_SOLID, TEST_CHASE, TEST_RAINBOW, TEST_FLASH = 0, 1, 2, 3, 4
PATTERN_NAMES = {TEST_OFF: "Aucun", TEST_SOLID: "Couleur / valeur unie",
                  TEST_CHASE: "Chenillard", TEST_RAINBOW: "Degrade", TEST_FLASH: "Flash"}

ALL_OUTPUTS = 255
TEST_TIMEOUT_S = 600

MODE_LED, MODE_DMX = 0, 1

# Formats pixel — miroir de PixelFormat_t / PixelFormat_BytesPerPixel dans
# Core/Inc/config.h. Informatif pour l'instant : le driver reel emet
# toujours en GRB 3 octets quel que soit ce reglage (voir web_ui.c).
PIXEL_FMT_RGB, PIXEL_FMT_GRB, PIXEL_FMT_BRG = 0, 1, 2
PIXEL_FMT_RGBW, PIXEL_FMT_GRBW, PIXEL_FMT_RGBWW = 3, 4, 5
PIXEL_FMT_COUNT = 6
PIXEL_FMT_NAMES = {0: "RGB", 1: "GRB", 2: "BRG", 3: "RGBW", 4: "GRBW", 5: "RGBWW"}
PIXEL_FMT_BYTES = {0: 3, 1: 3, 2: 3, 3: 4, 4: 4, 5: 5}


# ─────────────────────────────────────────────────────────────────────────
#  État simulé — équivalent de Config_Get() / WebUI_Stats_t / s_dmx_val[]
# ─────────────────────────────────────────────────────────────────────────

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = MODE_LED
        self.net_mode = 1  # statique
        self.ip = [2, 0, 0, 4]
        self.netmask = [255, 255, 255, 0]
        self.gateway = [2, 0, 0, 1]
        self.protocol = PROTO_SACN
        self.outputs = [
            {"enabled": True, "universe": i, "led_count": 120, "pixel_format": PIXEL_FMT_GRB}
            for i in range(MAX_OUTPUTS)
        ]
        self.artnet_packets = 0
        self.artnet_last_ms = 0
        self.artnet_last_universe = 0
        self.sacn_packets = 0
        self.sacn_last_ms = 0
        self.sacn_last_universe = 0
        self.dmx_val = [bytearray(DMX_SLOTS) for _ in range(MAX_OUTPUTS)]
        self.dmx_len = [0] * MAX_OUTPUTS
        self.dmx_ms = [0] * MAX_OUTPUTS
        self.test_active = False
        self.test_pattern = TEST_OFF
        self.test_output = ALL_OUTPUTS
        self.test_started = 0.0
        self.save_count = 0  # compteur de "Config_Save()" simulés

    def now_ms(self):
        return int(time.monotonic() * 1000)


S = State()


def simulate_traffic():
    """Simule un flux sACN qui peuple les canaux — pour voir /dmx et
    /groupes bouger sans matériel réel."""
    t = 0
    while True:
        time.sleep(0.5)
        with S.lock:
            S.sacn_packets += 1
            S.sacn_last_ms = S.now_ms()
            S.sacn_last_universe = S.outputs[0]["universe"]
            for i, out in enumerate(S.outputs):
                if not out["enabled"]:
                    continue
                buf = bytearray(DMX_SLOTS)
                for c in range(out["led_count"] * 3):
                    buf[c] = (int(127 + 127 * random.uniform(-1, 1) *
                                  __import__("math").sin(t / 10 + c / 20 + i)) & 0xFF)
                S.dmx_val[i] = buf
                S.dmx_len[i] = out["led_count"] * 3
                S.dmx_ms[i] = S.now_ms()
            if S.test_active and S.now_ms() - S.test_started >= TEST_TIMEOUT_S * 1000:
                S.test_active = False
        t += 1


# ─────────────────────────────────────────────────────────────────────────
#  Rendu — même structure que emit_header()/build_*() dans web_ui.c
# ─────────────────────────────────────────────────────────────────────────

CSS = """
body{font-family:system-ui,sans-serif;margin:0;background:#0f1115;color:#e8e8e8}
header{background:#1e88e5;padding:14px 20px;font-size:20px;font-weight:600}
nav{background:#161a20;padding:0 12px}
nav a{display:inline-block;color:#9fb4c8;padding:12px 16px;text-decoration:none;border-bottom:3px solid transparent}
nav a.on{color:#fff;border-bottom-color:#1e88e5}
nav a:hover{color:#fff}
main{padding:20px;max-width:820px}
h2{font-size:16px;color:#9fb4c8;margin:22px 0 8px;text-transform:uppercase;letter-spacing:.5px}
table{border-collapse:collapse;width:100%;margin:6px 0;background:#161a20;border-radius:8px;overflow:hidden}
td,th{border-bottom:1px solid #232a33;padding:9px 12px;text-align:left}
th{background:#1b2028;color:#9fb4c8;font-weight:600}
tr:last-child td{border-bottom:0}
input,select{background:#0f1115;color:#e8e8e8;border:1px solid #2a323d;padding:6px;border-radius:5px}
button{background:#1e88e5;color:#fff;border:0;padding:10px 22px;border-radius:6px;cursor:pointer;font-size:15px;font-weight:600}
button:hover{background:#1976d2}
.ok{color:#4caf50;font-weight:600}.off{color:#78828c}
.pill{display:inline-block;padding:2px 10px;border-radius:20px;font-size:13px;font-weight:600}
.pill.on{background:#14331c;color:#4caf50}.pill.no{background:#2a2020;color:#c86}
.card{background:#161a20;border-radius:8px;padding:14px 16px;margin:8px 0}
.card h3{margin:0 0 6px;font-size:15px;display:flex;align-items:center;gap:8px}
.card label{display:block;font-size:12px;color:#78828c;margin:8px 0 2px}
.warnbar{background:#3a2010;color:#f5b26b;border:1px solid #6b3a12;border-radius:6px;padding:10px 14px;margin:0 0 16px;font-weight:600}
.simbar{background:#122b1e;color:#7fd99d;border:1px solid #1d5c3a;border-radius:6px;padding:8px 14px;margin:0 0 16px;font-size:12px}
"""


def emit_header(title, refresh_s, active):
    banner = ""
    if S.test_active:
        banner = ("<div style=background:#c0392b;color:#fff;text-align:center;"
                  "padding:6px;font-weight:600;font-size:13px>"
                  "SEQUENCE DE TEST EN COURS — les sorties ne refletent pas le reseau</div>")
    refresh = f"<meta http-equiv=refresh content={refresh_s}>" if refresh_s > 0 else ""
    tabs = [("/", "Statut", 0), ("/flux", "Reception", 1), ("/dmx", "Canaux", 2),
            ("/groupes", "Sorties", 4), ("/test", "Test", 5), ("/config", "Configuration", 3)]
    nav = "".join(f'<a href={href} class={"on" if active == a else ""}>{label}</a>'
                  for href, label, a in tabs)
    return (f"<!doctype html><html><head><meta charset=utf-8>"
            f'<meta name=viewport content="width=device-width,initial-scale=1">{refresh}'
            f"<title>LEPIX Node - {title}</title><style>{CSS}</style></head><body>"
            f"{banner}"
            f'<div class=simbar>Simulateur local (tools/webserv_sim.py) — pas le firmware reel, '
            f'voir "Fidelite" dans le script</div>'
            f"<header>LEPIX Node</header><nav>{nav}</nav><main>")


FOOTER = "</main></body></html>"


def build_status():
    with S.lock:
        link = True
        n = emit_header("Statut", 0, 0)
        n += (f"<h2>Reseau</h2><table>"
              f"<tr><th>Lien</th><td class={'ok' if link else 'off'}>{'UP' if link else 'DOWN'}</td></tr>"
              f"<tr><th>Mode</th><td>{'DHCP' if S.net_mode == 0 else 'Statique'}</td></tr>"
              f"<tr><th>IP</th><td>{'.'.join(map(str, S.ip))}</td></tr>"
              f"<tr><th>Masque</th><td>{'.'.join(map(str, S.netmask))}</td></tr>"
              f"<tr><th>Passerelle</th><td>{'.'.join(map(str, S.gateway))}</td></tr>"
              f"</table>")
        n += (f"<h2>Entree</h2><table>"
              f"<tr><th>Protocole</th><td>{PROTO_NAMES[S.protocol]}</td></tr>"
              f"<tr><th>Art-Net</th><td>{S.artnet_packets} paquets</td></tr>"
              f"<tr><th>sACN</th><td>{S.sacn_packets} paquets</td></tr>"
              f"</table><p><a href=/flux style=color:#90caf9>Voir la reception en detail &rarr;</a></p>")
        n += "<h2>Sorties</h2><table><tr><th>#</th><th>Etat</th><th>Univers</th><th>LEDs</th></tr>"
        for i, o in enumerate(S.outputs):
            n += (f"<tr><td>{i+1}</td><td class={'ok' if o['enabled'] else 'off'}>"
                  f"{'ON' if o['enabled'] else 'off'}</td><td>{o['universe']}</td>"
                  f"<td>{o['led_count']}</td></tr>")
        n += "</table>" + FOOTER
        return n


def emit_flux_row(name, packets, last_uni, last_ms):
    ago = S.now_ms() - last_ms
    active = packets > 0 and ago < 5000
    if packets == 0:
        return (f"<tr><th>{name}</th><td><span class='pill no'>INACTIF</span></td>"
                f"<td>0</td><td>-</td><td>-</td></tr>")
    pill = "on" if active else "no"
    return (f"<tr><th>{name}</th><td><span class='pill {pill}'>"
            f"{'ACTIF' if active else 'silence'}</span></td>"
            f"<td>{packets}</td><td>{last_uni}</td><td>{ago/1000:.1f} s</td></tr>")


def build_flux():
    with S.lock:
        n = emit_header("Reception", 2, 1)
        n += ("<h2>Flux entrants</h2><table><tr><th>Protocole</th><th>Etat</th>"
              "<th>Paquets</th><th>Dernier univers</th><th>Derniere trame</th></tr>")
        n += emit_flux_row("Art-Net", S.artnet_packets, S.artnet_last_universe, S.artnet_last_ms)
        n += emit_flux_row("sACN (E1.31)", S.sacn_packets, S.sacn_last_universe, S.sacn_last_ms)
        n += ("</table><p style=color:#78828c;font-size:13px>"
              "Rafraichissement automatique toutes les 2 s. "
              "Un flux est ACTIF si une trame a ete recue depuis moins de 5 s.</p>" + FOOTER)
        return n


def build_dmx():
    with S.lock:
        n = emit_header("Canaux DMX", 0, 2)
        opts = "".join(
            f'<option value={i}>{i+1} — univers {o["universe"]}{"" if o["enabled"] else " (off)"}</option>'
            for i, o in enumerate(S.outputs))
        n += f"<h2>Matrice des canaux</h2><p>Sortie <select id=out>{opts}</select> <span id=inf class=off></span></p>"
        n += ('<canvas id=cv width=704 height=352 '
              'style="width:100%;background:#0a0c10;border-radius:8px"></canvas>'
              '<p id=tip class=off>Survoler une cellule pour lire canal et valeur.</p>'
              '<script>'
              "const cv=document.getElementById('cv'),cx=cv.getContext('2d');"
              "const sel=document.getElementById('out');let last=null;"
              "function draw(j){last=j;const W=32,H=16,cw=cv.width/W,ch=cv.height/H;"
              " cx.clearRect(0,0,cv.width,cv.height);"
              " for(let i=0;i<512;i++){"
              "  const v=i<j.len?parseInt(j.hex.substr(i*2,2),16):0;"
              "  const x=(i%W)*cw,y=Math.floor(i/W)*ch;"
              "  const g=cx.createLinearGradient(x,y,x,y+ch);"
              "  g.addColorStop(0,'rgb('+v+','+Math.round(v*.62)+','+Math.round(v*.18)+')');"
              "  g.addColorStop(1,'rgb('+Math.round(v*.35)+','+Math.round(v*.22)+',0)');"
              "  cx.fillStyle=g;cx.fillRect(x+1,y+1,cw-2,ch-2);}}"
              "async function poll(){try{"
              " const r=await fetch('/dmxdata'+sel.value);const j=await r.json();draw(j);"
              " document.getElementById('inf').textContent=j.len?"
              "  (j.len+' canaux — trame il y a '+(j.age/1000).toFixed(1)+' s'):"
              "  'aucune trame recue pour cette sortie';"
              "}catch(e){}setTimeout(poll,200);}"  # 5 Hz
              "cv.onmousemove=e=>{if(!last)return;const r=cv.getBoundingClientRect();"
              " const cx2=Math.floor((e.clientX-r.left)/r.width*32);"
              " const cy=Math.floor((e.clientY-r.top)/r.height*16);"
              " const i=cy*32+cx2;if(i<0||i>511)return;"
              " const v=i<last.len?parseInt(last.hex.substr(i*2,2),16):0;"
              " document.getElementById('tip').textContent='canal '+(i+1)+' = '+v;};"
              "poll();"
              '</script>')
        n += FOOTER
        return n


def build_dmxdata(idx):
    with S.lock:
        length = S.dmx_len[idx]
        age = (S.now_ms() - S.dmx_ms[idx]) if length else 0
        hexs = S.dmx_val[idx][:length].hex()
        return json.dumps({"out": idx + 1, "len": length, "age": age, "hex": hexs})


def build_groupes():
    with S.lock:
        is_dmx = S.mode == MODE_DMX
        n = emit_header("Sorties", 2, 4)
        n += f"<h2>Sorties ({'mode DMX filaire — 2 ports' if is_dmx else 'mode LED — 4 chaines WS2815'})</h2>"
        count = 2 if is_dmx else MAX_OUTPUTS
        for i in range(count):
            o = S.outputs[i]
            length = S.dmx_len[i]
            age = (S.now_ms() - S.dmx_ms[i]) if length else 0
            recent = length and age < 5000
            frame_desc = "aucune" if not length else f"il y a {age/1000:.1f} s"
            pf_row = ""
            if not is_dmx:
                pf = o["pixel_format"]
                pf_row = (f"<tr><th>Format pixel</th><td>{PIXEL_FMT_NAMES[pf]} "
                          f"({PIXEL_FMT_BYTES[pf]} octets/px)</td></tr>")
            n += (f"<div class=card><h3>Sortie {i+1}"
                  f"<span class='pill {'on' if o['enabled'] else 'no'}'>"
                  f"{'ACTIVE' if o['enabled'] else 'COUPEE'}</span>"
                  f"<span class='pill {'on' if recent else 'no'}'>"
                  f"{'TRAME RECENTE' if recent else 'SILENCE'}</span></h3>"
                  f"<table>"
                  f"<tr><th>Univers</th><td>{o['universe']}</td></tr>"
                  f"<tr><th>{'Port physique' if is_dmx else 'LEDs'}</th>"
                  f"<td>{i if is_dmx else o['led_count']}</td></tr>"
                  f"{pf_row}"
                  f"<tr><th>Protocole entree</th><td>{PROTO_NAMES[S.protocol]}</td></tr>"
                  f"<tr><th>Derniere trame</th><td>{frame_desc}</td></tr>"
                  f"</table>"
                  f"<p><a href='/dmx' style=color:#90caf9>Voir la matrice des canaux &rarr;</a></p></div>")
        n += ("<p style=color:#78828c;font-size:13px>Rafraichissement auto toutes les 2 s. "
              "Pour editer univers/LEDs/activation, voir <a href=/config style=color:#90caf9>"
              "Configuration</a>.</p>" + FOOTER)
        return n


def build_test():
    with S.lock:
        is_dmx = S.mode == MODE_DMX
        n = emit_header("Test", 2 if S.test_active else 0, 5)
        n += ("<h2>Sequence de test</h2><p style=color:#78828c>"
              "Injecte un motif directement sur les sorties, sans attendre de flux reseau. "
              "Coupure automatique apres 10 min d'inactivite operateur, par securite.</p>")
        if S.test_active:
            remaining = max(0, TEST_TIMEOUT_S - (S.now_ms() - S.test_started) // 1000)
            target = "toutes les sorties" if S.test_output == ALL_OUTPUTS else "une sortie"
            n += (f"<div class=warnbar>Test actif : {PATTERN_NAMES[S.test_pattern]} sur {target} — "
                  f"arret automatique dans {int(remaining)} s</div>"
                  f"<form action=/testctl method=get><input type=hidden name=action value=stop>"
                  f"<button type=submit style=background:#c0392b>Arreter le test</button></form>")
        n += ("<h2>Lancer un test</h2><form action=/testctl method=get>"
              "<input type=hidden name=action value=start>"
              "<label>Motif</label><select name=pattern>"
              "<option value=1>Couleur / valeur unie</option>"
              "<option value=2>Chenillard</option>"
              f"<option value=3 {'disabled' if is_dmx else ''}>Degrade (LED uniquement)</option>"
              "<option value=4>Flash</option></select>"
              "<label>Sortie / port cible</label><select name=out><option value=255>Toutes</option>")
        count = 2 if is_dmx else MAX_OUTPUTS
        for i in range(count):
            n += f"<option value={i}>{i+1} — univers {S.outputs[i]['universe']}</option>"
        n += ("</select><label>Couleur (motif Couleur unie, mode LED)</label>"
              "<input type=color name=rgb value=#ffffff>"
              "<p><button type=submit>Lancer</button></p></form>" + FOOTER)
        return n


def build_config():
    with S.lock:
        n = emit_header("Configuration", 0, 3)
        n += (f"<form action=/save method=get><h2>Reseau</h2><table>"
              f"<tr><th>Mode</th><td><select name=nm>"
              f"<option value=0 {'selected' if S.net_mode==0 else ''}>DHCP</option>"
              f"<option value=1 {'selected' if S.net_mode==1 else ''}>Statique</option></select></td></tr>"
              f"<tr><th>IP</th><td><input name=ip value={'.'.join(map(str,S.ip))}></td></tr>"
              f"<tr><th>Masque</th><td><input name=mk value={'.'.join(map(str,S.netmask))}></td></tr>"
              f"<tr><th>Passerelle</th><td><input name=gw value={'.'.join(map(str,S.gateway))}></td></tr>"
              f"</table>")
        n += ("<h2>Protocole</h2>"
              '<p style=color:#78828c;font-size:12px;margin:0 0 6px>'
              'Art-Net et sACN sont recus simultanement et fusionnes (HTP) — ce '
              'reglage est indicatif (affichage/fixture perte de signal), il ne '
              'coupe aucun des deux protocoles.</p>'
              f"<select name=pr>"
              f"<option value=0 {'selected' if S.protocol==0 else ''}>Art-Net</option>"
              f"<option value=1 {'selected' if S.protocol==1 else ''}>sACN</option>"
              f"<option value=2 {'selected' if S.protocol==2 else ''}>DMX (UART)</option></select>")
        n += ("<h2>Sorties</h2><table><tr><th>#</th><th>Active</th><th>Univers</th>"
              "<th>LEDs</th><th>Format pixel</th></tr>")
        for i, o in enumerate(S.outputs):
            pf_opts = "".join(
                f"<option value={f} {'selected' if o['pixel_format']==f else ''}>"
                f"{PIXEL_FMT_NAMES[f]} ({PIXEL_FMT_BYTES[f]} o/px)</option>"
                for f in range(PIXEL_FMT_COUNT))
            n += (f"<tr><td>{i+1}</td>"
                  f"<td><input type=checkbox name=e{i} {'checked' if o['enabled'] else ''}></td>"
                  f"<td><input name=u{i} value={o['universe']} size=6></td>"
                  f"<td><input name=l{i} value={o['led_count']} size=6></td>"
                  f"<td><select name=pf{i}>{pf_opts}</select></td></tr>")
        n += ("</table>"
              '<p style=color:#78828c;font-size:12px;margin:6px 0 0>'
              'Format pixel : ordre des canaux du flux DMX pour cette sortie. '
              'Valeur enregistree et affichee dans Sorties — le driver de sortie '
              'utilise pour l\'instant toujours GRB 3 octets quel que soit ce reglage.</p>')
        n += (f"<p><button type=submit>Enregistrer</button></p></form>"
              f"<p style=color:#78828c;font-size:12px>Config_Save() simule : {S.save_count} appel(s)</p>"
              + FOOTER)
        return n


# ─────────────────────────────────────────────────────────────────────────
#  CGI — équivalent de cgi_save() / cgi_testctl()
# ─────────────────────────────────────────────────────────────────────────

def cgi_save(params):
    with S.lock:
        if "nm" in params:
            S.net_mode = 1 if params["nm"][0] == "1" else 0
        for key, target in (("ip", S.ip), ("mk", S.netmask), ("gw", S.gateway)):
            if key in params:
                try:
                    parts = [int(x) for x in params[key][0].split(".")]
                    if len(parts) == 4:
                        target[:] = parts
                except ValueError:
                    pass
        if "pr" in params:
            try:
                S.protocol = int(params["pr"][0])
            except ValueError:
                pass
        for i, o in enumerate(S.outputs):
            o["enabled"] = f"e{i}" in params
            if f"u{i}" in params:
                try:
                    u = int(params[f"u{i}"][0])
                    o["universe"] = max(0, min(63999, u))
                except ValueError:
                    pass
            if f"l{i}" in params:
                try:
                    l = int(params[f"l{i}"][0])
                    o["led_count"] = max(0, min(WS2815_MAX_LEDS, l))
                except ValueError:
                    pass
            if f"pf{i}" in params:
                try:
                    pf = int(params[f"pf{i}"][0])
                    if 0 <= pf < PIXEL_FMT_COUNT:
                        o["pixel_format"] = pf
                except ValueError:
                    pass
        S.save_count += 1  # simule le Config_Save() differe (WebUI_Task)
    return "/config"


def cgi_testctl(params):
    with S.lock:
        action = params.get("action", [""])[0]
        if action == "stop":
            S.test_active = False
            return "/test"
        pattern = int(params.get("pattern", ["1"])[0])
        out = int(params.get("out", ["255"])[0])
        S.test_active = True
        S.test_pattern = pattern
        S.test_output = out if out != 255 else ALL_OUTPUTS
        S.test_started = S.now_ms()
    return "/test"


# ─────────────────────────────────────────────────────────────────────────
#  Serveur HTTP
# ─────────────────────────────────────────────────────────────────────────

ROUTES_HTML = {
    "/": build_status, "/index.html": build_status,
    "/flux": build_flux, "/flux.html": build_flux,
    "/dmx": build_dmx, "/dmx.html": build_dmx,
    "/groupes": build_groupes, "/groupes.html": build_groupes,
    "/test": build_test, "/test.html": build_test,
    "/config": build_config, "/config.html": build_config,
}
CGI_ROUTES = {"/save": cgi_save, "/testctl": cgi_testctl}


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silencieux ; utiliser --verbose pour le voir

    def do_GET(self):
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        params = parse_qs(parsed.query)

        if path.startswith("/dmxdata") and path[8:].isdigit() and int(path[8:]) < MAX_OUTPUTS:
            body = build_dmxdata(int(path[8:])).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if path in CGI_ROUTES:
            redirect = CGI_ROUTES[path](params)
            self.send_response(303)
            self.send_header("Location", redirect)
            self.end_headers()
            return

        if path in ROUTES_HTML:
            body = ROUTES_HTML[path]().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_response(404)
        self.end_headers()
        self.wfile.write(b"404 - non gere par le simulateur")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--mode", choices=("led", "dmx"), default="led",
                    help="simule le jumper PA5 (defaut : led)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    S.mode = MODE_DMX if args.mode == "dmx" else MODE_LED
    if args.verbose:
        Handler.log_message = http.server.BaseHTTPRequestHandler.log_message

    threading.Thread(target=simulate_traffic, daemon=True).start()

    with socketserver.ThreadingTCPServer(("127.0.0.1", args.port), Handler) as httpd:
        print(f"Simulateur Web UI LEPIX_NODE — http://127.0.0.1:{args.port}/")
        print(f"Mode simule : {'DMX (2 ports)' if S.mode == MODE_DMX else 'LED (4 sorties WS2815)'}")
        print("Ctrl-C pour arreter.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

# ─────────────────────────────────────────────────────────────────────────
# Fidélité au firmware — ce que ce simulateur NE reproduit PAS :
#
#   - Pas de contrainte mémoire : le firmware génère chaque page dans un
#     buffer RAM de 6 Ko (WEBUI_BUF_SIZE) partagé entre 2 connexions
#     (WEBUI_NUM_BUFS) ; un dépassement tronque silencieusement le HTML.
#     Ce script ne peut donc PAS détecter un dépassement de buffer — après
#     avoir ajouté du HTML/JS à une page, vérifier sa taille avec
#     `grep -A2 'define WEBUI_BUF_SIZE' Core/Src/web_ui.c` et estimer la
#     taille réelle (ex: len(build_xxx().encode())) avant de considérer
#     que "ça marche en local" == "ça marchera sur la carte".
#   - Pas de httpd raw LwIP : ce script utilise le serveur HTTP threadé de
#     la stdlib. Le comportement no-RTOS/mono-requête du firmware (accès
#     concurrents, buffers "coincés" récupérés après 15 s) n'est pas
#     reproduit.
#   - Le trafic DMX est un bruit sinusoïdal aléatoire, pas de vraies trames
#     Art-Net/sACN reçues.
#   - cgi_save() ici ne persiste rien sur disque (pas de FatFs/SD) et
#     n'applique aucun changement réseau réel (pas de MX_LWIP_ApplyNetworkConfig).
#
# Utiliser ce simulateur pour valider mise en page, navigation, JS de la
# matrice, cohérence des formulaires — puis valider le comportement réel
# (mémoire, timing, DMA) sur la carte flashee.
