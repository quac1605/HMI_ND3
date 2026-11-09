/* Joypad input driver: no HTTP, ROS, serial, or application-specific element IDs.
 * Browser API: globalThis.JoypadDriver. See ../README.md for the adapter contract.
 */
(function (root) {
  'use strict';

  const directions = ['forward', 'backward', 'left', 'right'];
  const wasdKeys = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD']);
  const arrowKeys = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);

  function normalizeAxes(x, y, deadzone = 0.05) {
    const magnitude = Math.hypot(x, y);
    if (![x, y, magnitude, deadzone].every(Number.isFinite) || deadzone < 0 || deadzone >= 1) {
      throw new RangeError('Axes must be finite; deadzone must be in [0, 1).');
    }
    if (magnitude < deadzone) return {x: 0, y: 0};
    const scale = Math.max(1, magnitude);
    return {x: x / scale, y: y / scale};
  }

  function validateSpeeds(speeds, limits) {
    const fields = {};
    for (const direction of directions) {
      const maximum = limits?.[direction === 'forward' || direction === 'backward'
        ? 'max_linear_speed' : 'max_angular_speed'];
      const value = speeds?.[direction];
      fields[direction] = Number.isFinite(maximum) && maximum > 0 &&
        Number.isFinite(value) && value >= 0 && value <= maximum;
    }
    return {valid: Object.values(fields).every(Boolean), fields};
  }

  // x positive = right, y positive = forward; ROS angular.z positive = left.
  // RPM limiting/rounding remains the responsibility of the robot-side adapter.
  function toCommand(axes, speeds, limits) {
    if (!validateSpeeds(speeds, limits).valid) throw new RangeError('Invalid speed settings.');
    const {x, y} = normalizeAxes(axes.x, axes.y, 0);
    return {
      v: y * speeds[y >= 0 ? 'forward' : 'backward'],
      w: -x * speeds[x <= 0 ? 'left' : 'right'],
    };
  }

  function editing(target) {
    return Boolean(target?.isContentEditable || target?.closest?.(
      'input, textarea, select, [contenteditable]:not([contenteditable="false"])'));
  }

  function create({surface, eventWindow, eventDocument, canStart, onStart, onMove, onStop,
    radius = 71, deadzone = 0.05}) {
    if (!surface || !eventWindow || !eventDocument ||
        ![canStart, onStart, onMove, onStop].every(fn => typeof fn === 'function') ||
        !Number.isFinite(radius) || radius <= 0) throw new TypeError('Invalid joypad adapter.');
    normalizeAxes(0, 0, deadzone);
    let holding = false, mode = null, pointer = null, axes = {x: 0, y: 0}, disposed = false;
    const keys = new Set();
    const listeners = [];

    function listen(target, name, handler) {
      target.addEventListener(name, handler);
      listeners.push([target, name, handler]);
    }
    function move(x, y) {
      axes = normalizeAxes(x, y, deadzone);
      onMove({...axes});
    }
    function stop(reason = 'Đã yêu cầu dừng.', beacon = false) {
      holding = false;
      mode = null;
      pointer = null;
      keys.clear();
      move(0, 0);
      onStop(reason, beacon);
    }
    function allowed() {
      return !disposed && !holding && eventDocument.hasFocus() && !eventDocument.hidden && canStart();
    }
    function start(nextMode) {
      if (!allowed()) return false;
      // Focus first: blur of the previously focused element must precede arming.
      surface.focus({preventScroll: true});
      holding = true;
      mode = nextMode;
      // Adapter owns asynchronous arming and must fence replies after stop.
      onStart();
      return holding;
    }
    function point(event) {
      const rect = surface.getBoundingClientRect();
      move((event.clientX - rect.left - rect.width / 2) / radius,
        -(event.clientY - rect.top - rect.height / 2) / radius);
    }
    function keyboardMove() {
      const map = mode === 'wasd' ? ['KeyD', 'KeyA', 'KeyW', 'KeyS']
        : ['ArrowRight', 'ArrowLeft', 'ArrowUp', 'ArrowDown'];
      move(Number(keys.has(map[0])) - Number(keys.has(map[1])),
        Number(keys.has(map[2])) - Number(keys.has(map[3])));
    }

    listen(surface, 'pointerdown', event => {
      if (pointer !== null || event.button !== 0 || !allowed()) return;
      event.preventDefault();
      if (!start('pointer')) return;
      pointer = event.pointerId;
      try { surface.setPointerCapture(pointer); }
      catch (_) { stop('Không giữ được joystick; yêu cầu dừng.'); return; }
      point(event);
    });
    listen(surface, 'pointermove', event => {
      if (event.pointerId === pointer && holding) point(event);
    });
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      listen(surface, name, event => {
        if (event.pointerId === pointer) stop('Đã thả joystick; yêu cầu dừng.');
      });
    }
    listen(eventWindow, 'keydown', event => {
      if (event.code === 'Escape') { event.preventDefault(); stop(); return; }
      if (event.ctrlKey || event.altKey || event.metaKey || event.isComposing || editing(event.target)) {
        if (holding && mode !== 'pointer') stop('Đã chuyển sang nhập liệu hoặc phím tắt; yêu cầu dừng.');
        return;
      }
      const wasd = wasdKeys.has(event.code);
      const arrows = event.code === 'Space' || arrowKeys.has(event.code);
      if (!wasd && !arrows) return;
      // Auto-repeat must never re-arm after stop, timeout, or reconnection.
      if (event.repeat) { if (holding && mode !== 'pointer') event.preventDefault(); return; }
      if (holding && mode === 'pointer') return;
      if (wasd) {
        if (mode && mode !== 'wasd') return;
        event.preventDefault();
        if (!holding && !start('wasd')) return;
        keys.add(event.code);
        keyboardMove();
      } else {
        if (mode === 'wasd' || !surface.contains(event.target)) return;
        event.preventDefault();
        if (event.code === 'Space' && !holding && !start('arrows')) return;
        if (mode === 'arrows' && holding) { keys.add(event.code); keyboardMove(); }
      }
    });
    listen(eventWindow, 'keyup', event => {
      if (mode === 'wasd' && wasdKeys.has(event.code)) {
        event.preventDefault();
        keys.delete(event.code);
        if (!keys.size) stop('Đã thả hết phím WASD; yêu cầu dừng.');
        else keyboardMove();
      } else if (mode === 'arrows') {
        if (event.code === 'Space') stop('Đã thả Space; yêu cầu dừng.');
        else if (arrowKeys.has(event.code)) { keys.delete(event.code); keyboardMove(); }
      }
    });
    listen(surface, 'blur', () => { if (holding) stop('Joystick mất focus; yêu cầu dừng.'); });
    listen(eventWindow, 'blur', () => stop('Cửa sổ mất focus; yêu cầu dừng.', true));
    listen(eventDocument, 'visibilitychange', () => {
      if (eventDocument.hidden) stop('Tab bị ẩn; yêu cầu dừng.', true);
    });
    listen(eventWindow, 'pagehide', () => stop('Rời trang; yêu cầu dừng.', true));
    listen(eventWindow, 'offline', () => stop('Mất mạng; yêu cầu dừng.', true));

    return Object.freeze({
      get holding() { return holding; },
      get mode() { return mode; },
      get axes() { return {...axes}; },
      command(speeds, limits) { return holding ? toCommand(axes, speeds, limits) : {v: 0, w: 0}; },
      stop,
      dispose() {
        if (disposed) return;
        disposed = true;
        for (const [target, name, handler] of listeners) target.removeEventListener(name, handler);
        stop('Đã ngắt joypad; yêu cầu dừng.');
      },
    });
  }

  root.JoypadDriver = Object.freeze({normalizeAxes, validateSpeeds, toCommand, create});
})(globalThis);
