// JavaScript-only safety tests: run `gjs tests/test_keyboard.js`.
// Executes the actual frontend with a fake DOM/HTTP transport; never contacts ROS/hardware.
const Gio = imports.gi.Gio;
const ByteArray = imports.byteArray;
const System = imports.system;
const read = path => ByteArray.toString(Gio.File.new_for_path(path).load_contents(null)[1]);
class Events {
  constructor() { this.listeners = new Map(); }
  addEventListener(name, listener) {
    if (!this.listeners.has(name)) this.listeners.set(name, []);
    this.listeners.get(name).push(listener);
  }
  emit(name, fields={}) {
    const event = {target:this, preventDefault(){this.prevented=true;}, ...fields};
    for (const listener of this.listeners.get(name) || []) listener(event);
    return event;
  }
}
class Element extends Events {
  constructor(id) {
    super(); this.id=id; this.style={}; this.value=''; this.isContentEditable=false;
    this.classList={add(){},remove(){}};
  }
  get valueAsNumber() { return this.value === '' ? NaN : Number(this.value); }
  setAttribute(name, value) { this[name]=value; }
  setCustomValidity(value) { this.validationMessage=value; }
  closest() { return ['forward','backward','left','right'].includes(this.id) || this.isContentEditable ? this : null; }
  contains(target) { return target===this; }
  focus() {
    if (document.activeElement!==this) {
      document.activeElement?.emit('blur');
      document.activeElement=this;
    }
  }
  setPointerCapture() {}
  getBoundingClientRect() { return {left:0,top:0,width:200,height:200}; }
}
const elements = new Map([...read('static/index.html').matchAll(/id="([^"]+)"/g)].map(m=>[m[1],new Element(m[1])]));
for (const id of ['forward','backward']) elements.get(id).value='0.10';
for (const id of ['left','right']) elements.get(id).value='0.30';
const body=new Element('body');
const document=new Events();
document.getElementById=id=>elements.get(id);
document.hidden=false; document.focused=true; document.hasFocus=()=>document.focused;
document.activeElement=body;
const window=new Events();
let clock=1000;
const performance={now:()=>clock};
const intervals=[];
const setInterval=fn=>intervals.push(fn);
const AbortSignal={timeout:()=>null};
class Blob { constructor(parts) { this.text=parts.join(''); } }
let requests=[], owner=false, serial=0, current={v:0,w:0,sent_at:null}, delayArm=false, releaseArm=null, failCommand=false;
const navigator={sendBeacon:(path,blob)=>{requests.push({path,data:JSON.parse(blob.text),beacon:true});owner=false;current={v:0,w:0,sent_at:1};return true;}};
function snapshot() {
  return {server_time:++serial, session:'test-session', ticket:'test-ticket', lease:'test-lease',
    demo:true,ready:true,occupied:owner,controlling:owner,
    config:{max_linear_speed:3,max_angular_speed:12.9,wheel_radius:.0825,wheel_base:.4,rpm_max:300,command_timeout:.5},
    command:{...current},driver:{reason:'simulated'},driver_age:0,
    encoder:{state:'fresh',age:0,stamp:1,received_at:1},wheels:[]};
}
async function fetch(path,options) {
  const data=JSON.parse(options.body);
  requests.push({path,data});
  if (path==='/api/command' && failCommand) throw new Error('simulated transport failure');
  if (path==='/api/arm') owner=true;
  if (path==='/api/command') current={v:data.v,w:data.w,sent_at:1};
  if (path==='/api/stop') {owner=false;current={v:0,w:0,sent_at:1};}
  const result=snapshot();
  if (path==='/api/arm' && delayArm) return new Promise(resolve=>{releaseArm=()=>resolve({ok:true,json:async()=>result});});
  return {ok:true,json:async()=>result};
}
const flush=async()=>{for(let i=0;i<16;i++)await Promise.resolve();};
const key=(name,code,extra={})=>window.emit(name,{code,target:document.activeElement,repeat:false,...extra});
function assert(value,message) {if(!value)throw new Error(message);}
function near(a,b,message) {assert(Math.abs(a-b)<1e-8,message+` (${a} != ${b})`);}
const last=path=>requests.filter(r=>r.path===path).at(-1);
async function mainTests(app) {
  await flush(); let passed=0;
  async function test(name,fn) {
    document.hidden=false;document.focused=true;document.activeElement=body;failCommand=false;
    app.stop();await flush();requests=[];
    await fn();passed++;print('PASS '+name);
  }
  await test('W/S/A/D map to the configured forward/backward/left/right commands',async()=>{
    for(const [code,v,w] of [['KeyW',.1,0],['KeyS',-.1,0],['KeyA',0,.3],['KeyD',0,-.3]]) {
      key('keydown',code);await flush();await app.poll();
      near(last('/api/command').data.v,v,code+' linear.x');near(last('/api/command').data.w,w,code+' angular.z');
      key('keyup',code);await flush();assert(last('/api/stop'),'release must send zero');
    }
  });
  await test('combined keys use joystick diagonal normalization; releasing one keeps the other',async()=>{
    key('keydown','KeyW');key('keydown','KeyA');await flush();await app.poll();
    near(last('/api/command').data.v,.1/Math.sqrt(2),'diagonal v');near(last('/api/command').data.w,.3/Math.sqrt(2),'diagonal w');
    key('keyup','KeyA');await app.poll();near(last('/api/command').data.v,.1,'remaining W');near(last('/api/command').data.w,0,'released A');
    const before=requests.filter(r=>r.path==='/api/stop').length;
    key('keyup','KeyW');await flush();assert(requests.filter(r=>r.path==='/api/stop').length===before+1,'last release must stop');
  });
  await test('opposite keys cancel each other',async()=>{
    key('keydown','KeyW');key('keydown','KeyS');await flush();await app.poll();
    near(last('/api/command').data.v,0,'W+S');key('keyup','KeyW');await app.poll();near(last('/api/command').data.v,-.1,'remaining S');key('keyup','KeyS');await flush();
  });
  await test('typing and Ctrl/Alt/Meta/IME never arm motion',async()=>{
    for(const id of ['forward','backward','left','right']) key('keydown','KeyW',{target:elements.get(id)});
    const editable=new Element('editor');editable.isContentEditable=true;key('keydown','KeyW',{target:editable});
    for(const modifier of ['ctrlKey','altKey','metaKey','isComposing'])key('keydown','KeyW',{[modifier]:true});
    await flush();assert(!last('/api/arm'),'typing must not arm');
  });
  await test('invalid speed input cannot arm',async()=>{
    elements.get('forward').value='-1';key('keydown','KeyW');await flush();assert(!last('/api/arm'),'invalid input');elements.get('forward').value='.1';
  });
  await test('Stop suppresses key auto-repeat until a new physical press',async()=>{
    key('keydown','KeyW');await flush();elements.get('stop').emit('click');await flush();
    const arms=requests.filter(r=>r.path==='/api/arm').length;
    key('keydown','KeyW',{repeat:true});await app.poll();assert(requests.filter(r=>r.path==='/api/arm').length===arms,'repeat rearmed');
    key('keyup','KeyW');key('keydown','KeyW');await flush();assert(requests.filter(r=>r.path==='/api/arm').length===arms+1,'new press should arm');key('keyup','KeyW');await flush();
  });
  await test('Escape stops and prevents held-key restart',async()=>{
    key('keydown','KeyD');await flush();key('keydown','Escape');await flush();assert(last('/api/stop'),'Escape stop');
    const arms=requests.filter(r=>r.path==='/api/arm').length;key('keydown','KeyD',{repeat:true});await flush();assert(requests.filter(r=>r.path==='/api/arm').length===arms,'Escape repeat');
  });
  await test('release during pending arm cannot produce a motion command',async()=>{
    delayArm=true;key('keydown','KeyW');await flush();assert(releaseArm,'arm was queued');key('keyup','KeyW');await flush();
    releaseArm();delayArm=false;await flush();await app.poll();assert(!last('/api/command'),'late arm must not resume');
  });
  await test('window blur and hidden tab send stop beacons',async()=>{
    key('keydown','KeyW');await flush();document.focused=false;window.emit('blur');await flush();assert(last('/api/stop')?.beacon,'blur beacon');
    document.focused=true;await app.poll();key('keydown','KeyW');await flush();document.hidden=true;document.emit('visibilitychange');await flush();assert(last('/api/stop')?.beacon,'hidden beacon');document.hidden=false;
  });
  await test('moving focus to input stops held keys',async()=>{
    key('keydown','KeyW');await flush();elements.get('forward').focus();await flush();assert(last('/api/stop'),'input focus must stop');
    requests=[];key('keydown','KeyW',{repeat:true});await flush();assert(!last('/api/arm'),'typing repeat must not resume');
  });
  await test('pointer and WASD cannot mix',async()=>{
    const joystick=elements.get('joystick');joystick.emit('pointerdown',{button:0,pointerId:7,clientX:100,clientY:29});await flush();
    key('keydown','KeyA');await app.poll();near(last('/api/command').data.v,.1,'pointer v');near(last('/api/command').data.w,0,'keyboard must not steer pointer');joystick.emit('pointerup',{pointerId:7});await flush();
    key('keydown','KeyW');await flush();joystick.emit('pointerdown',{button:0,pointerId:8,clientX:171,clientY:100});await app.poll();near(last('/api/command').data.w,0,'pointer must not steer keyboard');key('keyup','KeyW');await flush();
  });
  await test('Space + arrows remain available on focused joystick',async()=>{
    elements.get('joystick').focus();key('keydown','Space');key('keydown','ArrowUp');await flush();await app.poll();near(last('/api/command').data.v,.1,'arrow v');key('keyup','Space');await flush();assert(last('/api/stop'),'Space release stop');
  });
  await test('command connection failure stops and cannot auto-rearm',async()=>{
    key('keydown','KeyW');await flush();failCommand=true;await app.poll();await flush();assert(last('/api/stop'),'network error stop');
    const arms=requests.filter(r=>r.path==='/api/arm').length;key('keydown','KeyW',{repeat:true});await flush();assert(requests.filter(r=>r.path==='/api/arm').length===arms,'failure repeat');failCommand=false;
  });
  print(`${passed} keyboard/frontend tests passed; no hardware or HTTP connection used.`);
}
eval(read('static/app.js')+'\nmainTests({poll, command, stop}).catch(error=>{printerr(error.stack);System.exit(1);});');
