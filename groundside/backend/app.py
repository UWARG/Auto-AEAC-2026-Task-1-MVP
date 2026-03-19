import json
from datetime import datetime, timezone
from pathlib import Path
from flask import Flask, jsonify, request
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

DB_PATH = Path(__file__).with_name("data.json")


def load_db():
    with open(DB_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def save_db(data):
    with open(DB_PATH, "w", encoding="utf-8") as f:
        return json.dump(data, f, indent=3)


@app.route("/api/ackme")
def ack():
    return jsonify({"message": "test"})


@app.route("/api/captures", methods=["GET"])
def list_captures():
    db = load_db()
    return jsonify(db.get("captures", []))


@app.route("/api/captures", methods=["POST"])
def create_capture():
    payload = request.get_json(silent=True) or {}
    filename = str(payload.get("filename", "")).strip()
    desc = str(payload.get("desc", "")).strip()
    time_value = str(payload.get("time", "")).strip()

    if not filename or not desc:
        return jsonify({"message": "filename and desc are required"}), 400

    if not time_value:
        time_value = datetime.now(timezone.utc).replace(microsecond=0).isoformat()

    capture = {
        "time": time_value,
        "filename": filename,
        "desc": desc,
    }

    db = load_db()
    db.setdefault("captures", []).append(capture)
    save_db(db)

    return jsonify(capture), 201


@app.route("/api/db_reset", methods=["POST"])
def reset_db():
    db = load_db()
    db["captures"].clear()
    save_db(db)
    return jsonify({"message": "Database reset successfully"}), 200


if __name__ == "__main__":
    app.run(debug=True)
