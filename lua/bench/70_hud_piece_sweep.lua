-- HUD piece sweep (FEATURE-13 Phase 2, research/HUD_PIECES.md). Needs the menu's "Script writes" ON.
-- Hides one Gfx_MovablePieceID piece at a time through the active HUD settings slot, in memory only, and puts the
-- original bytes back when the piece changes, the rows are turned off, writes go off, or the scripts reload.
--   "HUD sweep" (toggle)       steps through the candidates every STEP seconds; the piece number is drawn on screen
--                              and logged. Bottom-right pieces come first (ammo hunt).
--   "HUD hide piece" (number)  hides exactly that piece while the sweep is off (-1 = none).
-- Do not change the game's HUD settings while this runs: the game could save the hidden state.
--
-- STATIC/LIVE layout: manager *(base+0x96A28B8); slot = u32 +0x108; records = ptr at mgr+(slot+0xB)*0x10;
-- record i = 0x1C bytes: enabled u32 +0x14, custom-enabled byte +0x18 (+0x19..+0x1B padding, kept).

local STEP = 4
local COUNT = 68
-- Right edge / bottom right first (#36 at 1,0.96 is the lead), then every other piece the HUD graphs queried.
-- Health bar group 7/12/45/63 is known, so it is skipped.
local ORDER = {36, 32, 0, 24, 49, 28, 26, 27, 31, 41,
               1, 2, 3, 4, 6, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19, 21, 22, 25, 29, 30, 33, 34, 35, 37, 38, 40,
               43, 46, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 64, 65, 67, 5, 9, 20, 23, 39, 42, 44, 47, 48, 66}

local hidden = nil  -- {id, addr, enabled, custom} while one piece is hidden
local sweep_start = nil

local function record(id)
  local base = avr.base()
  if not base or id < 0 or id >= COUNT then return nil end
  local mgr = avr.read_ptr(base + 0x96A28B8)
  if not mgr or mgr == 0 then return nil end
  local slot = avr.read_u32(mgr + 0x108)
  if not slot or slot >= 3 then return nil end
  local rec = avr.read_ptr(mgr + (slot + 0xB) * 0x10)
  if not rec or rec == 0 then return nil end
  return rec + id * 0x1C
end

-- Writes are refused while "Script writes" is off, so a hidden piece is only forgotten once its bytes went back;
-- otherwise the restore is retried every frame and a warning stays on screen.
local function restore()
  if not hidden then return true end
  if avr.write_u32(hidden.addr + 0x14, hidden.enabled) and avr.write_u32(hidden.addr + 0x18, hidden.custom) then
    avr.log(string.format("hud sweep: restored piece #%d", hidden.id))
    hidden = nil
    return true
  end
  return false
end

local function hide(id)
  if hidden and hidden.id == id then return end
  if not restore() then return end
  local addr = record(id)
  if not addr then return end
  local enabled, custom = avr.read_u32(addr + 0x14), avr.read_u32(addr + 0x18)
  if not enabled or not custom then return end
  hidden = {id = id, addr = addr, enabled = enabled, custom = custom}
  if not avr.write_u32(addr + 0x14, 0) then hidden = nil return end
  avr.write_u32(addr + 0x18, (custom & 0xFFFFFF00) | 1)
  avr.log(string.format("hud sweep: hiding piece #%d (was enabled=%d custom=%d) at t=%.1f", id, enabled, custom & 0xFF, avr.time()))
end

avr.on("frame", function()
  avr.toggle("HUD sweep", false)
  avr.tunable("HUD hide piece", -1, -1, COUNT - 1, 1)
  local auto = avr.value("HUD sweep")
  local manual = math.floor(avr.value("HUD hide piece") or -1)
  if not avr.writes() or (not auto and manual < 0) then
    sweep_start = nil
    if not restore() then
      avr.draw.screen_text(0.30, 0.30, string.format("HUD piece #%d still hidden: turn Script writes on to restore", hidden.id), 0xFF4040FF)
    end
    return
  end
  local id
  if auto then
    sweep_start = sweep_start or avr.time()
    local step = math.floor((avr.time() - sweep_start) / STEP)
    id = ORDER[(step % #ORDER) + 1]
  else
    sweep_start = nil
    id = manual
  end
  hide(id)
  if hidden then avr.draw.screen_text(0.45, 0.30, string.format("HUD piece #%d hidden", hidden.id), 0xFFFF40FF) end
end)

avr.on("unload", restore)
