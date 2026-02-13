import tkinter as tk
from tkinter import scrolledtext, messagebox
import subprocess
import os
import sys
import threading
import platform
import signal

processes = []

# Basis-Pfad (exe oder Script)
base_dir = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else os.path.dirname(os.path.abspath(__file__))

# PDF-Ordner
pdf_folder = os.path.join(base_dir, "data", "pdfs")
os.makedirs(pdf_folder, exist_ok=True)

# .env Pfad
env_path = os.path.join(base_dir, ".env")

def load_api_key():
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            for line in f:
                if line.startswith("OPENROUTER_API_KEY="):
                    return line.strip().split("=", 1)[1]
    return ""

def save_api_key(api_key):
    with open(env_path, "w") as f:
        f.write(f"OPENROUTER_API_KEY={api_key}\n")

def start_chainlit(api_key):
    save_api_key(api_key)

    try:
        proc = subprocess.Popen(
            [sys.executable, "-m", "chainlit", "run", "src/main.py"],
            cwd=base_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        )
        processes.append(proc)

        def read_output():
            for line in proc.stdout:
                output_text.insert(tk.END, line)
                output_text.see(tk.END)
            proc.stdout.close()

        threading.Thread(target=read_output, daemon=True).start()

    except Exception as e:
        messagebox.showerror("Fehler", f"Chainlit konnte nicht gestartet werden:\n{e}")

def on_start():
    api_key = api_entry.get().strip()
    if not api_key:
        messagebox.showwarning("Warnung", "Bitte API-Key eingeben!")
        return
    start_chainlit(api_key)

def on_close():
    for proc in processes:
        try:
            if proc.poll() is None:
                if os.name == 'nt':
                    proc.send_signal(signal.CTRL_BREAK_EVENT)
                    proc.kill()
                else:
                    proc.terminate()
        except:
            pass
    root.destroy()

def open_pdf_folder():
    try:
        if platform.system() == "Windows":
            os.startfile(pdf_folder)
        elif platform.system() == "Darwin":
            subprocess.call(["open", pdf_folder])
        else:
            subprocess.call(["xdg-open", pdf_folder])
    except Exception as e:
        messagebox.showerror("Fehler", f"PDF-Ordner konnte nicht geöffnet werden:\n{e}")

def toggle_console():
    if console_visible.get():
        output_frame.pack_forget()
        console_visible.set(False)
        toggle_button.config(text="Konsole anzeigen")
        root.geometry("720x230")
    else:
        output_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(0, 15))
        console_visible.set(True)
        toggle_button.config(text="Konsole ausblenden")
        root.geometry("720x520")

# ---------------- GUI ----------------

root = tk.Tk()
root.title("PDF Research Assistant")
root.geometry("720x520")
root.configure(bg="#f2f4f7")

# Titelbereich
header = tk.Frame(root, bg="#1f2933")
header.pack(fill=tk.X)

tk.Label(
    header,
    text="📚 PDF Research Assistant",
    font=("Segoe UI", 16, "bold"),
    fg="white",
    bg="#1f2933",
    pady=10
).pack()

tk.Label(
    header,
    text="Launcher für den KI-gestützten PDF Research Assistant mit OpenRouter API",
    font=("Segoe UI", 10),
    fg="#cbd5e1",
    bg="#1f2933",
    pady=2
).pack()

# Card
card = tk.Frame(root, bg="white", bd=0, highlightthickness=1, highlightbackground="#d1d5db")
card.pack(fill=tk.X, padx=15, pady=15)

tk.Label(card, text="OpenRouter API-Key", bg="white", font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=15, pady=(10, 2))

api_entry = tk.Entry(card, width=90, show="•", font=("Segoe UI", 10))
api_entry.pack(padx=15, pady=(0, 10), fill=tk.X)
api_entry.insert(0, load_api_key())

button_frame = tk.Frame(card, bg="white")
button_frame.pack(pady=(0, 10))

tk.Button(
    button_frame,
    text="▶ Start Chatbot",
    command=on_start,
    width=25,
    bg="#2563eb",
    fg="white",
    font=("Segoe UI", 10, "bold"),
    relief=tk.FLAT
).pack(side=tk.LEFT, padx=8)

tk.Button(
    button_frame,
    text="📂 PDF-Ordner öffnen",
    command=open_pdf_folder,
    width=20,
    bg="#e5e7eb",
    fg="#111827",
    font=("Segoe UI", 10),
    relief=tk.FLAT
).pack(side=tk.LEFT, padx=8)

console_visible = tk.BooleanVar(value=True)

toggle_button = tk.Button(
    root,
    text="Konsole ausblenden",
    command=toggle_console,
    bg="#e5e7eb",
    relief=tk.FLAT
)
toggle_button.pack(pady=(0, 5))

output_frame = tk.Frame(root)
output_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(0, 15))

output_text = scrolledtext.ScrolledText(
    output_frame,
    height=10,
    bg="#0f172a",
    fg="#e5e7eb",
    insertbackground="white",
    font=("Consolas", 9)
)
output_text.pack(fill=tk.BOTH, expand=True)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()