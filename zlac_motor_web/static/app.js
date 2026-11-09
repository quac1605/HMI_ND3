'use strict';
const $ = id => document.getElementById(id);
const inputs = ['forward', 'backward', 'left', 'right'].map($);
let session = null, seq = 0, ticket = null, lease = null, state = null;
let busy = false;
let lastResponse = 0, latestServerTime = 0, stopGeneration = 0;
const speeds = () => Object.fromEntries(inputs.map(input => [input.id, input.valueAsNumber]));
const fmt = (n, digits = 4) => n == null ? '—' : `${n < -0.0000001 ? '−' : '+'}${Math.abs(n).toFixed(digits)}`;
const timeLabel = n => n == null ? '—' : new Date(n * 1000).toLocaleTimeString('vi-VN', {hour12:false});
function validate() {
  if (!state?.config) return false;
  const cfg = state.config;
  const validation = JoypadDriver.validateSpeeds(speeds(), cfg);
  inputs.forEach((input, i) => {
    input.max = i < 2 ? cfg.max_linear_speed : cfg.max_angular_speed;
    const ok = validation.fields[input.id];
    input.setCustomValidity(ok ? '' : `Nhập số từ 0 đến ${Number(input.max).toFixed(3)}.`);
  });
  $('input-error').textContent = validation.valid ? '' : 'Tốc độ phải hữu hạn, không âm và nằm trong giới hạn.';
  return validation.valid;
}
// The web adapter displays input intent and handles transport. The joypad module
// owns input events, normalization, sign conventions, and speed validation.
function command() {
  if (!joypad.holding || !validate()) return {v:0,w:0};
  return joypad.command(speeds(), state.config);
}
function move({x, y}) {
  $('stick').style.transform = `translate(${x * 71}px,${-y * 71}px)`;
  const c = command();
  $('requested').textContent = `v ${fmt(c.v,3)} · ω ${fmt(c.w,3)}`;
}
function apply(s) {
  lastResponse = performance.now();
  if (s.server_time < latestServerTime) return;
  latestServerTime = s.server_time;
  state = s; ticket = s.ticket;
  $('mode').textContent = s.demo ? 'DEMO / GIẢ LẬP' : 'ROS 2 / PHẦN CỨNG';
  $('mode').className = `badge ${s.demo ? 'demo' : ''}`;
  $('demo-banner').hidden = !s.demo;
  $('connection').textContent = '● Cầu nối đã kết nối';
  $('footer-mode').textContent = s.demo ? 'DEMO · Không phát /cmd_vel · Không truy cập phần cứng' : 'Web → ROS 2 → Driver · Không truy cập RS485 từ web';
  $('joystick').setAttribute('aria-disabled', String(!s.ready || !validate()));
  $('cmd-v').textContent = fmt(s.command.v, 3); $('cmd-w').textContent = fmt(s.command.w, 3);
  $('command-status').textContent = s.controlling ? 'Đang phát liên tục' : s.command.sent_at ? 'Lệnh cuối' : 'Chưa phát lệnh';
  $('command-time').textContent = `${s.demo ? 'Giả lập' : 'Phát lên ROS'}: ${timeLabel(s.command.sent_at)}`;
  const driverFresh = s.driver && s.driver_age < 0.5;
  $('driver-status').textContent = s.demo ? 'Giám sát giả lập' : !driverFresh ? 'Giám sát ROS: mất / quá hạn' :
    `Giám sát: ${s.driver.reason} · Chưa có ACK phần cứng`;
  const enc = s.encoder;
  const labels = {fresh:s.demo ? 'ENCODER GIẢ LẬP' : 'ENCODER MỚI', stale:'ENCODER QUÁ HẠN', missing:'CHƯA CÓ ENCODER', invalid:'ENCODER KHÔNG HỢP LỆ'};
  $('encoder-status').textContent = labels[enc.state];
  $('encoder-status').className = `badge ${enc.state === 'fresh' ? s.demo ? 'demo' : '' : 'danger'}`;
  $('encoder-time').textContent = `Nhận mẫu: ${timeLabel(enc.received_at)}`;
  $('encoder-age').textContent = `Tuổi dữ liệu: ${enc.age == null ? '—' : (enc.age * 1000).toFixed(0)+' ms'}`;
  $('encoder-stamp').textContent = `Timestamp ${s.demo ? 'giả lập' : 'ROS'}: ${enc.stamp?.toFixed(6) ?? '—'}${enc.error ? ' · '+enc.error : ''}`;
  s.wheels.forEach((wheel,i) => {
    const side = i === 0 ? 'left' : 'right';
    $(side+'-expected').textContent = fmt(wheel.mps);
    $(side+'-actual').textContent = $(side+'-measured').textContent = fmt(wheel.measured);
    $(side+'-error').textContent = fmt(wheel.error);
    const full = s.config.rpm_max * 2 * Math.PI / 60 * s.config.wheel_radius;
    [['expected',wheel.mps],['measured',wheel.measured]].forEach(([kind,value]) => {
      const p = Math.min(50, Math.abs(value ?? 0) / full * 50);
      $(side+'-'+kind+'-bar').style.width = p+'%';
      $(side+'-'+kind+'-bar').style.left = ((value ?? 0) < 0 ? 50-p : 50)+'%';
    });
  });
  $('clamp-warning').hidden = !s.wheels.some(w => w.limited);
  $('rpm-detail').textContent = 'RPM tính → RPM gửi: '+s.wheels.map((w,i) => `${i ? 'Phải':'Trái'} ${fmt(w.raw_rpm,2)} → ${fmt(w.sent_rpm,0)}`).join('   /   ');
  if (s.config) {
    const c = s.config;
    $('limits').textContent = `Giới hạn: ${c.max_linear_speed} m/s · ${c.max_angular_speed.toFixed(3)} rad/s. Kết hợp hai trục có thể chạm giới hạn RPM.`;
    $('radius').textContent = c.wheel_radius+' m'; $('base').textContent = c.wheel_base+' m';
    $('rpm').textContent = c.rpm_max+' RPM'; $('watchdog').textContent = c.command_timeout*1000+' ms';
  }
  if (!joypad.holding) $('action').textContent = !validate() && s.config ? 'Sửa giá trị tốc độ trước khi điều khiển.' : !s.ready ? 'Chưa sẵn sàng: kiểm tra driver, encoder và chỉ một nguồn /cmd_vel.' : s.occupied ? 'Một tab khác đang điều khiển.' : 'Sẵn sàng. Giữ joystick hoặc phím WASD để chạy.';
  if (joypad.holding && (!s.ready || (lease && !s.controlling))) stop('Quyền điều khiển mất hoặc dữ liệu quá hạn.');
}
async function api(path, data={}, timeout=350) {
  const response = await fetch('/api/'+path, {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({session, ...data}), signal:AbortSignal.timeout(timeout), cache:'no-store'});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Cầu nối từ chối lệnh.');
  return result;
}
function stop(reason='Đã yêu cầu dừng.', beacon=false) {
  joypad.stop(reason, beacon);
}
function sendStop(reason, beacon) {
  ++stopGeneration;
  lease = null;
  $('joystick').classList.remove('active');
  $('action').textContent = reason;
  if (!session) return;
  const body = {session, seq:++seq};
  if (beacon) {
    navigator.sendBeacon('/api/stop', new Blob([JSON.stringify(body)], {type:'application/json'}));
  } else {
    api('stop',body).then(apply).catch(() => {
      $('action').textContent = 'Không xác nhận được lệnh dừng. Watchdog sẽ yêu cầu 0; kiểm tra encoder / E-stop.';
    });
  }
}
async function begin() {
  if (!joypad.holding) return false;
  const generation = stopGeneration;
  $('joystick').classList.add('active');
  try {
    const result = await api('arm',{seq:++seq,ticket});
    if (!joypad.holding || generation !== stopGeneration) return false;
    lease = result.lease; apply(result);
    $('action').textContent = joypad.mode === 'wasd' ? 'Đang điều khiển WASD. Thả hết phím để dừng.' : 'Giữ để tiếp tục. Thả để dừng.';
    return true;
  } catch (err) { stop(err.message); return false; }
}
const joypad = JoypadDriver.create({
  surface: $('joystick'), eventWindow: window, eventDocument: document,
  canStart: () => Boolean(session && state?.ready && validate()),
  onStart: begin, onMove: move, onStop: sendStop,
});
$('stop').addEventListener('click',()=>stop());
inputs.forEach(input=>input.addEventListener('input',()=>{if(joypad.holding)stop('Đổi tốc độ; nhấn lại joystick để chạy.');validate();}));
$('dropout').addEventListener('click',async()=>{try{apply(await api('demo-dropout'));}catch(e){stop(e.message);}});
async function poll() {
  if (busy || !session || document.hidden) return;
  busy = true;
  try {
    const s = joypad.holding && lease ? await api('command',{seq:++seq,ticket,lease,...command()}) : await api('state');
    apply(s);
  } catch (err) {
    if (joypad.holding) stop(err.message);
    $('connection').textContent = '● Mất kết nối cầu nối';
    $('joystick').setAttribute('aria-disabled','true');
  } finally { busy = false; }
}
setInterval(poll,50);
setInterval(()=>{
  if (lastResponse && performance.now()-lastResponse > 350) {
    if (joypad.holding) stop('Cầu nối không phản hồi; yêu cầu dừng.',true);
    if(state) state.ready = false;
    $('connection').textContent='● Mất kết nối cầu nối';
    $('joystick').setAttribute('aria-disabled','true');
    $('encoder-status').textContent='DỮ LIỆU KHÔNG CÒN MỚI';
    $('encoder-status').className='badge danger';
    for(const side of ['left','right']) for(const key of ['actual','measured','error']) $(side+'-'+key).textContent='—';
  }
},50);
(async()=>{try{const s=await api('session',{},1500);session=s.session;apply(s);}catch(err){$('action').textContent=err.message+' Tải lại trang khi cầu nối đã chạy.';}})();
