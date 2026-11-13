#include "../../src/vr/menu_pointer.hpp"
#include <cstdlib>
using namespace menu_pointer;
void check(bool ok) { if (!ok) std::abort(); }
bool near(float a,float b) { return std::fabs(a-b)<.002f; }
struct Pad { float lt=0,rt=0; unsigned short buttons=0; };
int main() {
  const float id[4]={0,0,0,1}, panel[3]={0,1,-2}, from[3]={0,1,0};
  float u=0,v=0,t=0;
  // Straight ahead: panel centre at the panel's distance.
  { const float d[3]={0,0,-1}; check(hit_quad(from,d,id,panel,2,1,&u,&v,&t)); check(near(u,.5f)&&near(v,.5f)&&near(t,2)); }
  // Right and up go to larger u and v_up.
  { const float o[3]={.5f,1.25f,0}, d[3]={0,0,-1}; check(hit_quad(o,d,id,panel,2,1,&u,&v,&t)); check(near(u,.75f)&&near(v,.75f)); }
  // Off the panel, pointing away, and from behind miss.
  { const float o[3]={1.5f,1,0}, d[3]={0,0,-1}; check(!hit_quad(o,d,id,panel,2,1,&u,&v,&t)); }
  { const float d[3]={0,0,1}; check(!hit_quad(from,d,id,panel,2,1,&u,&v,&t)); }
  { const float o[3]={0,1,-3}, d[3]={0,0,1}; check(!hit_quad(o,d,id,panel,2,1,&u,&v,&t)); }
  // A panel turned 90 degrees to the left stands at -x and faces +x.
  { const float s=std::sqrt(.5f), q[4]={0,s,0,s}, p[3]={-2,1,0}, d[3]={-1,0,0}; check(hit_quad(from,d,q,p,2,1,&u,&v,&t)); check(near(u,.5f)&&near(t,2)); }
  { const float s=std::sqrt(.5f), q[4]={0,s,0,s}, p[3]={-2,1,0}, o[3]={0,1,-.5f}, d[3]={-1,0,0}; check(hit_quad(o,d,q,p,2,1,&u,&v,&t)); check(near(u,.75f)); }

  Selector s;
  const bool both[2]={true,true}, right[2]={false,true}, left[2]={true,false}, none[2]={false,false};
  const float idle[2]={0,0}, rt[2]={0,1}, lt[2]={1,0};
  // The active hand's trigger is A while held, with one press event.
  auto o=step(s,both,idle); check(o.hand==1&&!o.a&&!o.press&&o.suppress);
  o=step(s,both,rt); check(o.a&&o.press);
  o=step(s,both,rt); check(o.a&&!o.press);
  o=step(s,none,rt); check(o.a&&o.suppress);   // A held off the panel until released
  o=step(s,none,idle); check(!o.a&&!o.suppress);
  // Other hand's trigger: takes over, no click until the next pull.
  o=step(s,both,lt); check(o.hand==0&&!o.a&&o.suppress);
  o=step(s,both,lt); check(!o.a);
  o=step(s,both,idle); check(!o.a);
  o=step(s,both,lt); check(o.hand==0&&o.a&&o.press);
  o=step(s,both,idle); check(!o.a);
  // Active hand off the panel, other hand on it: follows without a pull.
  o=step(s,right,idle); check(o.hand==1&&o.hit&&!o.a);
  o=step(s,right,rt); check(o.a&&o.press);
  o=step(s,left,rt); check(o.hand==1&&o.a);    // no handover mid-click
  o=step(s,left,idle); check(!o.a);
  // Pulled off the panel and swept onto it: no click; the trigger stays with the game off the panel.
  Selector s2;
  o=step(s2,none,rt); check(!o.a&&!o.suppress);
  o=step(s2,right,rt); check(!o.a&&!o.press);
  // Partial pull between thresholds does not chatter.
  Selector s3; const float half[2]={0,.5f}, high[2]={0,.7f};
  o=step(s3,both,half); check(!o.a);
  o=step(s3,both,high); check(o.a);
  o=step(s3,both,half); check(o.a);

  // Pad output and staleness.
  publish(true,.25f,.75f,true,true,1000);
  Pad pad; pad.lt=pad.rt=1;
  apply(&pad,1100); check(pad.lt==0&&pad.rt==0&&pad.buttons==0x1000);
  check(cursor(&u,&v,1100)&&near(u,.25f)&&near(v,.75f));
  Pad old; old.lt=1;
  apply(&old,2000); check(old.lt==1&&old.buttons==0);
  check(!cursor(&u,&v,2000));
  g_hover_at.store(5000);
  check(interactive(5000+kHoverHoldMs-1)&&!interactive(5000+kHoverHoldMs));
  return 0;
}
