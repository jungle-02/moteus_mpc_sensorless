namespace moteus {
struct GitInfo {
  GitInfo();
  static const char* const g_hash;
};
GitInfo::GitInfo() {}
const char* const GitInfo::g_hash = "manual_build_v6.0";
}