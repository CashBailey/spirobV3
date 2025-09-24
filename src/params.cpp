#include "params.hpp"

void ParamsStore::begin() {
  prefs.begin("pf360", false);
}

String ParamsStore::keyPrefix(char label, const char* key) {
  String s; s.reserve(8);
  s += label; s += "_"; s += key;
  return s;
}

void ParamsStore::loadServo(char label, ServoParams& sp) {
  String kp = keyPrefix(label, "Kp");
  String ki = keyPrefix(label, "Ki");
  String kd = keyPrefix(label, "Kd");
  String db = keyPrefix(label, "DBx10");
  String mf = keyPrefix(label, "MinF");
  String sc = keyPrefix(label, "SpCap");
  String mn = keyPrefix(label, "Fmin");
  String mx = keyPrefix(label, "Fmax");

  sp.Kp = prefs.getFloat(kp.c_str(), sp.Kp);
  sp.Ki = prefs.getFloat(ki.c_str(), sp.Ki);
  sp.Kd = prefs.getFloat(kd.c_str(), sp.Kd);
  sp.deadband_x10 = prefs.getShort(db.c_str(), sp.deadband_x10);
  sp.min_force    = prefs.getShort(mf.c_str(), sp.min_force);
  sp.speed_cap_rpm= prefs.getShort(sc.c_str(), sp.speed_cap_rpm);
  sp.feed_min_us  = prefs.getUShort(mn.c_str(), sp.feed_min_us);
  sp.feed_max_us  = prefs.getUShort(mx.c_str(), sp.feed_max_us);
}

void ParamsStore::saveServo(char label, const ServoParams& sp) {
  prefs.putFloat(keyPrefix(label, "Kp").c_str(), sp.Kp);
  prefs.putFloat(keyPrefix(label, "Ki").c_str(), sp.Ki);
  prefs.putFloat(keyPrefix(label, "Kd").c_str(), sp.Kd);
  prefs.putShort(keyPrefix(label, "DBx10").c_str(), sp.deadband_x10);
  prefs.putShort(keyPrefix(label, "MinF").c_str(), sp.min_force);
  prefs.putShort(keyPrefix(label, "SpCap").c_str(), sp.speed_cap_rpm);
  prefs.putUShort(keyPrefix(label, "Fmin").c_str(), sp.feed_min_us);
  prefs.putUShort(keyPrefix(label, "Fmax").c_str(), sp.feed_max_us);
}

void ParamsStore::loadGlobal(GlobalParams& gp) {
  gp.telemetry_rate_hz = prefs.getUShort("G_TelHz", gp.telemetry_rate_hz);
  gp.start_on_hold     = prefs.getBool("G_StartHold", gp.start_on_hold);
  gp.baud              = prefs.getULong("G_Baud", gp.baud);
  gp.guard_window_deg  = prefs.getShort("G_Guard", gp.guard_window_deg);
}

void ParamsStore::saveGlobal(const GlobalParams& gp) {
  prefs.putUShort("G_TelHz", gp.telemetry_rate_hz);
  prefs.putBool("G_StartHold", gp.start_on_hold);
  prefs.putULong("G_Baud", gp.baud);
  prefs.putShort("G_Guard", gp.guard_window_deg);
}
