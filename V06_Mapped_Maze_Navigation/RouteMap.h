#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Encoder-space, directed pose graph. An edge is an actually driven sequence,
// not an invented reverse traversal. Positions are estimates, not surveyed data.
namespace route {
constexpr int MAX_NODES = 256, MAX_EDGES = 512, MAX_COMMANDS = 2048;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float MATCH_DISTANCE = 180.0f; // ~1.6-2.3 cm with user's calibration
constexpr float MATCH_ANGLE = 12.0f * PI_F / 180.0f;
constexpr float MIN_REVISIT_TRAVEL = 2400.0f;
enum Kind : uint8_t { RAW = 0, CRUISE = 1, CHECK = 2 };
struct Command {
  uint32_t left, right;
  int16_t speedLeft, speedRight;
  uint8_t kind, mask;
  uint16_t reserved;
};
static_assert(sizeof(Command) == 16, "Stored command format changed");
struct Pose { float x, y, yaw; };
constexpr float absolute(float v) { return v<0?-v:v; }
constexpr float angle(float v) {
  while (v > PI_F) v -= 2 * PI_F;
  while (v < -PI_F) v += 2 * PI_F;
  return v;
}
inline void integrate(Pose &p, float left, float right) {
  // 520 counts on each wheel in opposite directions is a calibrated 90 deg.
  float a = (right - left) * PI_F / (4.0f * 520.0f);
  float d = (left + right) * 0.5f;
  float scale = fabsf(a) < 0.00001f ? 1.0f : sinf(a * 0.5f) / (a * 0.5f);
  p.x += d * scale * cosf(p.yaw + a * 0.5f);
  p.y += d * scale * sinf(p.yaw + a * 0.5f);
  p.yaw = angle(p.yaw + a);
}
struct Node { Pose pose; float lastTravel; uint8_t mask; };
struct Edge { uint16_t from, to, first, count; float cost; int8_t turnQuarters; };
struct Graph {
  Node nodes[MAX_NODES] = {};
  Edge edges[MAX_EDGES] = {};
  uint16_t nodeCount = 0, edgeCount = 0;
  void clear() { nodeCount = edgeCount = 0; }
  // Require identical relative wall signature AND heading. Ambiguous candidates
  // stay separate. Never snap the odometer to a guessed match.
  constexpr int landmark(Pose p, uint8_t mask, float travel, bool allowMatch) {
    int candidate = -1;
    if (allowMatch) {
      for (int i = 0; i < nodeCount; ++i) {
        const Node &n = nodes[i];
        float dx = n.pose.x - p.x, dy = n.pose.y - p.y;
        bool centre = (mask & 0x80) && mask != 255;
        if (n.mask != mask || travel - n.lastTravel < MIN_REVISIT_TRAVEL ||
            (!centre && absolute(angle(n.pose.yaw - p.yaw)) > MATCH_ANGLE) ||
            dx * dx + dy * dy > MATCH_DISTANCE * MATCH_DISTANCE) continue;
        if (candidate >= 0) {
          const Node &other=nodes[candidate];
          float sx=n.pose.x-other.pose.x, sy=n.pose.y-other.pose.y;
          // Several headings at the same turning centre are one physical place.
          // Two spatially different plausible places are ambiguous: don't merge.
          if(!centre || sx*sx+sy*sy>(MATCH_DISTANCE/2)*(MATCH_DISTANCE/2)) {
            candidate=-2; break;
          }
          if(absolute(angle(n.pose.yaw-p.yaw)) >= absolute(angle(other.pose.yaw-p.yaw))) continue;
        }
        candidate = i;
      }
    }
    if (candidate >= 0 && absolute(angle(nodes[candidate].pose.yaw-p.yaw)) <= MATCH_ANGLE) {
      nodes[candidate].lastTravel = travel; return candidate;
    }
    if (nodeCount == MAX_NODES) return -1;
    nodes[nodeCount] = {p, travel, mask};
    int created=nodeCount++;
    if(candidate >= 0) {
      float delta=angle(p.yaw-nodes[candidate].pose.yaw);
      float quarters=delta/(PI_F/2);
      int q=(int)(quarters+(quarters>=0?0.5f:-0.5f));
      // A physical junction centre supports in-place transitions between observed
      // headings. Only synthesize a calibrated cardinal turn, never translation.
      if(q && absolute(angle(delta-q*PI_F/2))<=MATCH_ANGLE) {
        if(edgeCount+2>MAX_EDGES) return -1;
        edges[edgeCount++]={(uint16_t)candidate,(uint16_t)created,0,0,520.0f*(q<0?-q:q),(int8_t)q};
        edges[edgeCount++]={(uint16_t)created,(uint16_t)candidate,0,0,520.0f*(q<0?-q:q),(int8_t)-q};
      }
    }
    return created;
  }
  constexpr bool connect(int from, int to, int first, int count, float cost) {
    if (from < 0 || to < 0 || from>=nodeCount || to>=nodeCount || first < 0 || count < 0 ||
        first + count > MAX_COMMANDS || edgeCount == MAX_EDGES) return false;
    edges[edgeCount++] = {(uint16_t)from, (uint16_t)to,
      (uint16_t)first, (uint16_t)count, cost > 0 ? cost : 0.001f, 0};
    return true;
  }
  // Dijkstra minimizes measured wheel travel, including turn effort. Thus this
  // optimizes a distance/turn-effort estimate, not measured race time.
  constexpr int shortest(int start, int goal, uint16_t *result, int capacity) const {
    if (start < 0 || goal < 0 || start >= nodeCount || goal >= nodeCount) return -1;
    float dist[MAX_NODES] = {}; int previous[MAX_NODES] = {}; bool done[MAX_NODES] = {};
    for (int i = 0; i < nodeCount; ++i) { dist[i] = 1e30f; previous[i] = -1; }
    dist[start] = 0;
    for (int iteration = 0; iteration < nodeCount; ++iteration) {
      int u = -1;
      for (int i = 0; i < nodeCount; ++i)
        if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
      if (u < 0 || dist[u] >= 1e30f) break;
      if (u == goal) break;
      done[u] = true;
      for (int e = 0; e < edgeCount; ++e) {
        const Edge &edge = edges[e];
        if (edge.from == u && dist[u] + edge.cost < dist[edge.to]) {
          dist[edge.to] = dist[u] + edge.cost; previous[edge.to] = e;
        }
      }
    }
    if (dist[goal] >= 1e30f) return -1;
    int n = 0;
    for (int v = goal; v != start;) {
      if (n == capacity || previous[v] < 0) return -1;
      int e = previous[v]; result[n++] = e; v = edges[e].from;
    }
    for (int i = 0; i < n / 2; ++i) {
      uint16_t t = result[i]; result[i] = result[n - 1 - i]; result[n - 1 - i] = t;
    }
    return n;
  }
};
inline uint32_t checksum(const void *data, unsigned size) {
  const uint8_t *p = static_cast<const uint8_t *>(data);
  uint32_t hash = 2166136261u;
  while (size--) { hash ^= *p++; hash *= 16777619u; }
  return hash;
}
constexpr bool validCommand(const Command &c) {
  if (c.reserved || c.mask > 7 || c.kind > CHECK) return false;
  if (c.kind == CHECK) return !c.left && !c.right && !c.speedLeft && !c.speedRight;
  if (c.kind == CRUISE) return c.left > 0 && c.left <= 10000000 &&
    !c.right && !c.speedLeft && !c.speedRight;
  return c.left <= 100000 && c.right <= 100000 && (c.left || c.right) &&
    c.speedLeft >= -255 && c.speedLeft <= 255 &&
    c.speedRight >= -255 && c.speedRight <= 255 &&
    (!c.left || c.speedLeft != 0) && (!c.right || c.speedRight != 0);
}
} // namespace route
