// Standalone GJS tests: no web server, ROS, serial, or motor hardware.
const Gio = imports.gi.Gio;
const ByteArray = imports.byteArray;
const System = imports.system;
const root = Gio.File.new_for_path(System.programPath).get_parent().get_parent();
eval(ByteArray.toString(root.get_child('src/joypad_driver.js').load_contents(null)[1]));
const {normalizeAxes, validateSpeeds, toCommand, create} = JoypadDriver;
const speeds = {forward: 0.2, backward: 0.1, left: 0.6, right: 0.3};
const limits = {max_linear_speed: 3, max_angular_speed: 12.9};
function assert(value, message) { if (!value) throw new Error(message); }
function near(value, expected) { assert(Math.abs(value - expected) < 1e-10, `${value} != ${expected}`); }
function rejects(fn) {
  let threw = false;
  try { fn(); } catch (error) { threw = error instanceof RangeError || error instanceof TypeError; }
  assert(threw, 'Invalid data must be rejected');
}
let passed = 0;
function test(name, fn) { fn(); passed++; print('PASS ' + name); }

class Events {
  constructor() { this.listeners = new Map(); }
  addEventListener(name, fn) {
    if (!this.listeners.has(name)) this.listeners.set(name, new Set());
    this.listeners.get(name).add(fn);
  }
  removeEventListener(name, fn) { this.listeners.get(name)?.delete(fn); }
  emit(name, fields = {}) {
    const event = {target: this, preventDefault() {}, ...fields};
    for (const fn of this.listeners.get(name) || []) fn(event);
  }
}
function fixture() {
  const surface = new Events(), win = new Events(), doc = new Events();
  const calls = {start: 0, stop: [], move: []};
  doc.hidden = false; doc.focused = true; doc.hasFocus = () => doc.focused;
  surface.focus = () => {};
  surface.contains = target => target === surface;
  surface.setPointerCapture = () => {};
  surface.getBoundingClientRect = () => ({left: 10, top: 20, width: 200, height: 200});
  const f = {surface, win, doc, calls, ready: true};
  f.driver = create({surface, eventWindow: win, eventDocument: doc,
    canStart: () => f.ready, onStart: () => { calls.start++; },
    onMove: axes => calls.move.push(axes), onStop: (reason, beacon) => calls.stop.push({reason, beacon})});
  f.pointer = (name, extra = {}) => surface.emit(name,
    {button: 0, pointerId: 7, clientX: 110, clientY: 49, ...extra});
  return f;
}

test('radial deadzone, boundary and proportional motion', () => {
  near(normalizeAxes(.02, -.02).x, 0);
  near(normalizeAxes(.05, 0).x, .05);
  const axes = normalizeAxes(.3, .4); near(axes.x, .3); near(axes.y, .4);
});
test('diagonal and over-travel preserve angle within the unit circle', () => {
  const axes = normalizeAxes(3, 4); near(axes.x, .6); near(axes.y, .8);
  near(normalizeAxes(-1, 1).x, -1 / Math.sqrt(2));
});
test('independent forward/backward/left/right speeds and ROS signs', () => {
  for (const [x, y, v, w] of [[0, .5, .1, 0], [0, -.5, -.05, 0], [-.5, 0, 0, .3], [.5, 0, 0, -.15]]) {
    const cmd = toCommand({x, y}, speeds, limits); near(cmd.v, v); near(cmd.w, w);
  }
  const cmd = toCommand({x: -1, y: 1}, speeds, limits);
  near(cmd.v, .2 / Math.sqrt(2)); near(cmd.w, .6 / Math.sqrt(2));
});
test('invalid axes and invalid deadzones are rejected', () => {
  for (const value of [NaN, Infinity, -Infinity, '1', null]) rejects(() => normalizeAxes(value, 0));
  for (const value of [-.1, 1, NaN]) rejects(() => normalizeAxes(0, 0, value));
});
test('finite nonnegative speed limits are enforced for every direction', () => {
  assert(validateSpeeds(speeds, limits).valid, 'valid settings');
  assert(validateSpeeds({...speeds, forward: 3, backward: 0}, limits).valid, 'inclusive bounds');
  for (const direction of Object.keys(speeds)) {
    for (const value of [-.1, NaN, Infinity, '', undefined, 100]) {
      const bad = {...speeds, [direction]: value};
      assert(!validateSpeeds(bad, limits).fields[direction], direction);
      rejects(() => toCommand({x: 0, y: 0}, bad, limits));
    }
  }
  for (const cfg of [null, {}, {...limits, max_linear_speed: 0}, {...limits, max_angular_speed: Infinity}]) {
    assert(!validateSpeeds(speeds, cfg).valid, 'bad config');
  }
});
test('pointer coordinates produce proportional commands and clamp over-travel', () => {
  const f = fixture(); f.pointer('pointerdown');
  assert(f.driver.holding && f.driver.mode === 'pointer', 'pointer must own input');
  near(f.driver.command(speeds, limits).v, .2);
  f.pointer('pointermove', {clientX: 145.5, clientY: 120});
  near(f.driver.command(speeds, limits).w, -.15);
  f.pointer('pointermove', {clientX: -600, clientY: 120});
  near(f.driver.command(speeds, limits).w, .6);
  f.pointer('pointerup'); near(f.driver.command(speeds, limits).v, 0);
  assert(!f.driver.holding && f.calls.stop.length === 1, 'release stop');
});
test('second touch cannot steer or release the active touch', () => {
  const f = fixture(); f.pointer('pointerdown');
  for (const name of ['pointerdown', 'pointermove', 'pointerup', 'pointercancel']) {
    f.pointer(name, {pointerId: 8, clientX: 181, clientY: 120});
  }
  assert(f.calls.start === 1 && f.driver.holding, 'first touch retained');
  near(f.driver.command(speeds, limits).v, .2); near(f.driver.command(speeds, limits).w, 0);
});
test('pointer cancel and lost capture reset all axes', () => {
  for (const name of ['pointercancel', 'lostpointercapture']) {
    const f = fixture(); f.pointer('pointerdown'); f.pointer(name);
    assert(!f.driver.holding && f.calls.stop.length === 1, name);
    near(f.driver.axes.x, 0); near(f.driver.axes.y, 0);
    f.pointer('pointermove'); assert(!f.driver.holding, 'move cannot restart');
  }
});
test('failure to capture pointer stops immediately', () => {
  const f = fixture(); f.surface.setPointerCapture = () => { throw new Error('capture failed'); };
  f.pointer('pointerdown'); assert(!f.driver.holding && f.calls.stop.length === 1, 'capture stop');
});
test('unready, unfocused, hidden and secondary-button input cannot start', () => {
  const f = fixture(); f.ready = false; f.pointer('pointerdown');
  f.ready = true; f.doc.focused = false; f.pointer('pointerdown');
  f.doc.focused = true; f.doc.hidden = true; f.pointer('pointerdown');
  f.doc.hidden = false; f.pointer('pointerdown', {button: 2});
  assert(f.calls.start === 0 && !f.driver.holding, 'start interlocks');
});
test('offline, pagehide, blur and hidden tab request beacon stop', () => {
  for (const name of ['offline', 'pagehide', 'blur', 'visibilitychange']) {
    const f = fixture(); f.pointer('pointerdown');
    if (name === 'visibilitychange') { f.doc.hidden = true; f.doc.emit(name); }
    else f.win.emit(name);
    assert(!f.driver.holding && f.calls.stop[0].beacon, name);
  }
});
test('dispose stops once and removes event listeners; exported axes cannot be mutated', () => {
  const f = fixture(); f.pointer('pointerdown');
  const axes = f.driver.axes; axes.y = -1; near(f.driver.axes.y, 1);
  f.driver.dispose(); f.driver.dispose(); f.pointer('pointerdown');
  f.win.emit('keydown', {code: 'KeyW'});
  assert(!f.driver.holding && f.calls.start === 1 && f.calls.stop.length === 1, 'dispose cleanup');
});
print(`${passed} standalone joypad tests passed; no hardware used.`);
