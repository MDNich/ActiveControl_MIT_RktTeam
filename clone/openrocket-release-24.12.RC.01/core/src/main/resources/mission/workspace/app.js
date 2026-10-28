import {state,$,esc,get,set,changed,status,request,action,commit,draftAction,useMission,initialize} from './api.js';
import {field,section,attachFields,renderFlight,renderEnsemble} from './forms.js';
import {renderSites} from './sites.js';
import {renderVideo} from './video.js';
const descriptions={site:'URRG launch and station locations · enter the missing elevations.',flight:'Complete flight configuration, with immutable inputs for each run.',telemetry:'Packets come from the flight computer or the standard Zephyrus source.',video:'One logical camera per stream, with explicit receiver routes.',connections:'Separate station identities and one command owner per receiver.',run:'Resolve inputs, execute a mission, and keep its results together.'};
function show(page){state.page=page;document.querySelectorAll('.page').forEach(el=>el.hidden=el.id!=='page-'+page);document.querySelectorAll('nav button').forEach(el=>{el.classList.toggle('active',el.dataset.page===page);el.setAttribute('aria-current',el.dataset.page===page?'page':'false');});$('description').textContent=descriptions[page];if(page==='site')renderSites();}
function defaults(){
  const mission=state.mission,telemetry=mission.telemetry;
  telemetry.noise??={gpsHorizontalSigmaM:0,gpsVerticalSigmaM:0,barometerSigmaM:0};telemetry.gpsRateHz??=10;telemetry.barometerRateHz??=20;
  for(const site of mission.sites.filter(s=>s.id!=='launch-site')){
    if(!telemetry.links.some(l=>l.receiverId===site.id))telemetry.links.push({receiverId:site.id,packetLossFraction:0,delayMs:0,jitterMs:0,bitErrorProbability:0,uartBaud:0,outages:[],directional:false,beamWidthDeg:60});
  }
  for(const station of mission.stationBindings)station.targetVehicleId??=mission.vehicleBindings[0].id;
  mission.execution.clockMode??='lockstep';mission.outputs.filename??='mission.rktmission';mission.outputs.recordEncodedVideo??=false;
}
function render(){
  defaults();$('title').value=state.mission.identity.title;$('revision').textContent='Revision '+state.mission.identity.revision;$('draft-state').textContent=state.dirty?'Unsaved changes':'Draft saved locally';$('change-status').textContent=state.dirty?'Changes are not yet applied.':'Applied locally. Save or export a portable setup to keep a reusable copy.';
  renderSites();renderFlight();renderTelemetry();renderVideo();renderConnections();renderRun();show(state.page);
}
function outages(path,values){return `<label>Outage windows · seconds<input data-outages="${path}" value="${esc(values.map(v=>`${Number(v.startUs)/1e6}-${Number(v.endUs)/1e6}`).join(', '))}" placeholder="12-15, 40-42"><span class="field-note">Start-end pairs, separated by commas.</span></label>`;}
function bindOutages(root){root.querySelectorAll('[data-outages]').forEach(el=>el.onchange=()=>{try{el.setCustomValidity('');const windows=el.value.trim()?el.value.split(',').map((part,i)=>{const match=part.trim().match(/^(-?\d+(?:\.\d+)?)\s*[-–]\s*(-?\d+(?:\.\d+)?)$/);if(!match||Number(match[2])<=Number(match[1]))throw Error('Use increasing start-end time pairs, such as 12-15, 40-42.');return {id:'outage-'+(i+1),startUs:String(Math.round(Number(match[1])*1e6)),endUs:String(Math.round(Number(match[2])*1e6))};}):[];set(el.dataset.outages,windows);}catch(error){el.setCustomValidity(error.message);el.reportValidity();status(error.message);}});}
function renderTelemetry(){
  let html=section('Packet source',[
    field('telemetry.mode','Source',{choices:[['standard','Standard Zephyrus · no custom FC'],['custom-fc','Custom flight computer']]}),field('telemetry.rateHz','Standard transmit rate · Hz',{type:'number',min:.1,max:1000,readonly:get('telemetry.mode')==='custom-fc'}),field('telemetry.fallback','FC without a declared format',{choices:[['legacy','Use legacy Zephyrus format'],['require-format','Require a declared format']]})
  ],'Custom flight computers own their transmit timing and packet layout. Choose the FC file and its mappings in Rocket & flight.');
  html+=section('Standard source observations',[
    field('telemetry.gpsRateHz','GPS acquisition · Hz',{type:'number',min:.1,max:1000}),field('telemetry.barometerRateHz','Barometer acquisition · Hz',{type:'number',min:.1,max:1000}),field('telemetry.noise.gpsHorizontalSigmaM','GPS horizontal noise σ · m',{type:'number',min:0}),field('telemetry.noise.gpsVerticalSigmaM','GPS vertical noise σ · m',{type:'number',min:0}),field('telemetry.noise.barometerSigmaM','Barometric height noise σ · m',{type:'number',min:0})
  ],'Measurements are held between acquisitions. Custom FC sensor settings come from its design. Standard packets mark unmodeled hardware channels unavailable.');
  html+='<div class="form-section"><h2>Receiver links</h2><p class="hint">Each receiver has an independent loss/noise stream. Software serial has no UART pacing unless a baud rate is selected.</p><div class="rows">';
  state.mission.telemetry.links.forEach((link,i)=>{
    const p=`telemetry.links.${i}.`;html+=`<details class="row-card" ${i===0?'open':''}><summary>${link.receiverId==='launch'?'Base station':'Away '+esc(link.receiverId.slice(2))}</summary><div class="field-grid">${field(p+'packetLossFraction','Packet loss · %',{type:'number',scale:100,min:0,max:100})}${field(p+'delayMs','Delivery delay · ms',{type:'number',min:0})}${field(p+'jitterMs','Delay jitter σ · ms',{type:'number',min:0})}${field(p+'bitErrorProbability','Bit error probability',{type:'number',min:0,max:1})}${field(p+'uartBaud','UART pacing · baud',{type:'number',min:0,step:1,note:'0 = software serial; typical hardware profile 115200.'})}${field(p+'directional','Directional antenna qualification')}${field(p+'beamWidthDeg','Illustrative beam width · °',{type:'number',min:.01,max:360})}${outages(p+'outages',link.outages||[])}</div></details>`;
  });html+='</div></div>';$('telemetry-fields').innerHTML=html;attachFields($('telemetry-fields'));bindOutages($('telemetry-fields'));
}
function renderConnections(){
  let html='<div class="form-section"><h2>Telemetry UI on this computer</h2><p class="hint">Application locations are machine preferences. Imported setups cannot select or start executables.</p><div class="rows">';
  for(const [kind,name] of [['sourceProject','Source project'],['pythonExecutable','Python interpreter'],['packagedApp','Packaged application']])html+=`<div class="row-card"><h3>${name}</h3><div class="path-value">${esc(state.preferences[kind]||'Not selected')}</div><button data-location="${kind}">Choose ${name.toLowerCase()}</button></div>`;
  html+='</div></div><div class="form-section"><h2>Station roles</h2><p class="hint">Base has no local pointer. Each Away telemetry endpoint has one command owner.</p><div class="rows">';
  state.mission.stationBindings.forEach((station,i)=>{
    const p=`stationBindings.${i}.`;html+=`<div class="row-card"><h3>${station.siteId==='launch'?'Base':'Away '+esc(station.siteId.slice(2))} · ${station.role==='video'?'Video':'Telemetry'}</h3><div class="field-grid">${field(p+'owner','Command ownership',{choices:station.role==='video'?[['observer','Video application']]:[['headless-policy','Headless station policy'],['interactive-ui','Interactive Telemetry UI'],['observer','Observer UI only']]})}${field(p+'targetVehicleId','Telemetry target',{choices:state.mission.vehicleBindings.map(v=>[v.id,v.id])})}</div></div>`;
  });
  html+='</div><div class="checks">'+['gs1','gs2','gs3','gs4'].map(site=>`<label><input type="checkbox" data-video-role="${site}" ${state.mission.stationBindings.some(b=>b.siteId===site&&b.role==='video')?'checked':''}>Away ${site.slice(2)} video application</label>`).join('')+'</div></div>';
  $('connection-fields').innerHTML=html;attachFields($('connection-fields'));
  document.querySelectorAll('[data-location]').forEach(b=>b.onclick=()=>action(async()=>{const result=await request('preferences',{kind:b.dataset.location});if(!result.cancelled){state.preferences=result;renderConnections();status('Application location saved for this computer.');}}));
  document.querySelectorAll('[data-video-role]').forEach(input=>input.onchange=()=>{const site=input.dataset.videoRole;state.mission.stationBindings=state.mission.stationBindings.filter(s=>s.siteId!==site||s.role!=='video');if(input.checked)state.mission.stationBindings.push({id:site+'-video',siteId:site,role:'video',owner:'observer',targetVehicleId:state.mission.vehicleBindings[0].id});changed();renderConnections();});
}
function renderRun(){
  let html=section('Execution',[
    field('execution.mode','Mission mode',{choices:[['ground-link','Ground Link Analysis · standard flight'],['coupled','Coupled Mission Simulation · custom FC'],['interactive','Paced application rehearsal']]}),field('execution.clockMode','Clock',{choices:[['lockstep','Deterministic · as fast as possible'],['paced','Paced · real time']]}),field('execution.seed','Mission seed',{note:'Unsigned 64-bit integer; independent of the flight seed.'}),field('execution.deliveryTailUs','Final delivery tail · µs',{note:'0–60,000,000 µs; default 2 seconds.'}),field('outputs.filename','Result filename'),field('outputs.recordEncodedVideo','Also save encoded video')
  ]);
  html+=renderEnsemble();
  if(state.mission.migration)html+=section('Imported preview review',[field('migration.reviewed','I have reviewed and applied the imported preview settings')],`${state.mission.migration.issues?.length||0} source fields need review. The original preview values are preserved in the saved configuration.`)+`<details><summary>Preview findings</summary>${(state.mission.migration.issues||[]).map(i=>`<div class="finding"><strong>${esc(i.path)}</strong><p>${esc(i.message)}</p></div>`).join('')}</details>`;
  $('run-fields').innerHTML=html;attachFields($('run-fields'));
  if(!state.resolved)$('start').disabled=true;
}
async function validate(){
  await commit();const result=await draftAction('validate');
  $('findings').innerHTML=result.issues.length?result.issues.map(i=>`<div class="finding"><strong>${esc(i.path)}</strong><p>${esc(i.message)}</p></div>`).join(''):'<p class="good">Configuration inputs are valid.</p>';
  if(result.configurationValid){const resolved=await draftAction('resolve');state.resolved=resolved.hash;$('start').disabled=!result.runtimeAvailable;
    $('findings').innerHTML+=result.runtimeAvailable?'<p class="hint">Inputs are frozen and ready. Start mission when you are ready.</p>':'<p class="hint">Mission runtime is unavailable. Check the OpenRocket installation before starting.</p>';}
  status(result.configurationValid?'Configuration checked.':'Correct the listed fields before starting.');
}
let pollTimer;
async function poll(){if(!state.job)return;try{const job=await request('jobs/'+state.job.id);state.job=job;$('job').hidden=false;$('job').textContent=JSON.stringify(job,null,2);$('pause').disabled=job.status!=='running';$('resume').disabled=job.status!=='paused';$('cancel').disabled=!['starting','running','paused'].includes(job.status);if(!['complete','complete-with-failures','cancelled','failed'].includes(job.status))pollTimer=setTimeout(poll,1000);}catch(error){status('Run status unavailable: '+error.message);pollTimer=setTimeout(poll,3000);}}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>show(b.dataset.page));
$('title').onchange=()=>{set('identity.title',$('title').value);};
$('apply').onclick=()=>action(commit);
$('save').onclick=()=>action(async()=>{const result=await draftAction('save');if(result.saved)status('Saved '+result.saved+'.');});
$('export').onclick=()=>action(async()=>{const result=await draftAction('export');if(result.saved)status('Exported '+result.saved+' with frozen rocket inputs.');});
$('load').onclick=()=>action(async()=>{await commit();const mission=await request('drafts/load',{});if(!mission.cancelled){useMission(mission);status('Configuration loaded. Applications remain disconnected.');}});
for(const [id,current] of [['current-rocket',true],['choose-rocket',false]])$(id).onclick=()=>action(async()=>{const result=await draftAction('rocket',{current});if(result.cancelled)return;$('simulation-choice').innerHTML=result.simulations.map(s=>`<option value="${s.index}">${esc(s.name)}</option>`).join('');$('simulation-choice').hidden=false;$('select-flight').hidden=false;status('Choose the saved flight settings to load.');});
$('select-flight').onclick=()=>action(async()=>{const result=await draftAction('flight',{simulation:Number($('simulation-choice').value)});useMission(result);status('Rocket, motor curves and flight settings frozen with this draft.');});
$('validate').onclick=()=>action(validate);
$('start').onclick=()=>action(async()=>{await commit();if(!state.resolved)await validate();if(!state.resolved||$('start').disabled)return;const job=await request('jobs',{draftId:state.mission.identity.missionId,expectedRevision:state.mission.identity.revision,resolvedHash:state.resolved,idempotencyKey:crypto.randomUUID()});state.job=job;clearTimeout(pollTimer);await poll();});
for(const name of ['pause','resume','cancel'])$(name).onclick=()=>action(async()=>{await request('jobs/'+state.job.id+'/'+name,{});clearTimeout(pollTimer);await poll();});
document.addEventListener('mission-loaded',render);document.addEventListener('mission-render',render);
document.addEventListener('field-changed',event=>{
  if(event.detail==='telemetry.mode'){state.mission.execution.mode=get('telemetry.mode')==='custom-fc'?'coupled':'ground-link';renderTelemetry();renderRun();}
  if(event.detail==='execution.mode'&&get('execution.mode')==='interactive'){state.mission.execution.clockMode='paced';renderRun();}
});
action(async()=>{await initialize();status('Mission workspace connected.');});
