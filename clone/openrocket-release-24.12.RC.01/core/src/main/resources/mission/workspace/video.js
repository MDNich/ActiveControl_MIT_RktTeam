import {state,$,esc,changed} from './api.js';
import {field,attachFields,section} from './forms.js';
let selected='';
export function renderVideo(){
  const video=state.mission.video;video.frameRateHz??=30;video.width??=960;video.height??=540;video.lossFraction??=0;video.delayMs??=0;video.analogNoiseFraction??=0;video.outages??=[];
  $('video-fields').innerHTML=section('Sources & delivery',[
    field('video.preset','Vehicle preset',{choices:[['Balius','Balius'],['Iris','Iris'],['Custom','Custom']]}),
    `<label>Digital streams<input id="digital-count" type="number" min="0" max="32" step="1" value="${video.streams.filter(s=>s.kind==='digital').length}"></label>`,
    `<label>Analog streams<input id="analog-count" type="number" min="0" max="32" step="1" value="${video.streams.filter(s=>s.kind==='analog').length}"></label>`,
    field('video.frameRateHz','Frames per second',{type:'number',min:1,max:60}),field('video.width','Slate width · px',{type:'number',min:160,max:3840,step:2}),field('video.height','Slate height · px',{type:'number',min:90,max:2160,step:2}),field('video.lossFraction','Delivery loss · %',{type:'number',scale:100,min:0,max:100}),field('video.delayMs','Delivery delay · ms',{type:'number',min:0}),field('video.analogNoiseFraction','Illustrative analog noise · %',{type:'number',scale:100,min:0,max:100})
  ],'Headless missions record logical frames and slate assets. Paced connections use the application video path; publisher capacity is checked before start.');
  attachFields($('video-fields'));
  for(const kind of ['digital','analog'])$(kind+'-count').onchange=()=>{
    const input=$(kind+'-count');if(!input.checkValidity()){input.reportValidity();return;}
    const count=Number(input.value),existing=video.streams.filter(s=>s.kind===kind),other=video.streams.filter(s=>s.kind!==kind);
    for(let i=existing.length;i<count;i++)existing.push({id:kind+'-'+(i+1),vehicleId:state.mission.vehicleBindings[0].id,kind,receivers:['gs1','gs2','gs3','gs4']});
    video.streams=[...other,...existing.slice(0,count)];video.preset='Custom';changed();renderVideo();
  };
  if(!video.streams.some(s=>s.id===selected))selected=video.streams[0]?.id||'';
  const ordinals={digital:0,analog:0};
  $('stream-list').innerHTML=video.streams.length?video.streams.map((stream,i)=>{
    const ordinal=++ordinals[stream.kind];
    return `<div class="stream-card ${selected===stream.id?'selected':''}"><div class="stream-heading"><strong>${stream.kind==='digital'?'Digital':'Analog'} ${ordinal}</strong><button data-preview="${esc(stream.id)}">Preview</button></div>${field(`video.streams.${i}.vehicleId`,'Vehicle',{choices:state.mission.vehicleBindings.map(v=>[v.id,v.id])})}<div class="checks">${['gs1','gs2','gs3','gs4'].map(site=>`<label><input type="checkbox" data-receiver="${i}:${site}" ${stream.receivers.includes(site)?'checked':''}>Away ${site.slice(2)}</label>`).join('')}</div><p class="hint">${esc(stream.id)}</p></div>`;
  }).join(''):'<div class="information">No video streams. Telemetry and pointer simulation remain available.</div>';
  attachFields($('stream-list'));
  $('stream-list').querySelectorAll('[data-preview]').forEach(b=>b.onclick=()=>{selected=b.dataset.preview;renderVideo();});
  $('stream-list').querySelectorAll('[data-receiver]').forEach(input=>input.onchange=()=>{const [i,site]=input.dataset.receiver.split(':'),stream=video.streams[Number(i)];stream.receivers=stream.receivers.filter(r=>r!==site);if(input.checked)stream.receivers.push(site);changed();});
  slate();
}
function slate(){
  const canvas=$('slate'),ctx=canvas.getContext('2d'),stream=state.mission.video.streams.find(s=>s.id===selected);
  ctx.fillStyle='#10282b';ctx.fillRect(0,0,960,540);ctx.fillStyle='#77d3bd';ctx.fillRect(0,0,960,12);ctx.font='bold 25px sans-serif';ctx.fillText('MISSION SIMULATION',42,69);
  ctx.fillStyle='#ffffff';ctx.font='bold 70px sans-serif';const ordinal=stream?state.mission.video.streams.filter(s=>s.kind===stream.kind).findIndex(s=>s.id===stream.id)+1:0;
  ctx.fillText(stream?`${stream.kind==='digital'?'Digital':'Analog'} ${ordinal}`:'No video',42,182,875);
  ctx.font='36px sans-serif';ctx.fillText(state.mission.identity.title,42,260,875);ctx.fillStyle='#b3cbc5';ctx.font='25px sans-serif';ctx.fillText(stream?.vehicleId||'Add a stream to preview its slate',42,318,875);
  ctx.fillText('Run & member assigned on start',42,391,875);ctx.font='21px sans-serif';ctx.fillText(state.mission.flight.filename||'Choose a rocket .ork',42,443,875);ctx.font='16px monospace';ctx.fillText(state.mission.identity.missionId,42,496,875);
}
document.addEventListener('field-changed',event=>{
  if(event.detail==='video.preset'){
    const preset=state.mission.video.preset;if(preset==='Custom'){renderVideo();return;}
    const stages=state.mission.flight.editor?.stages||[];
    state.mission.vehicleBindings=preset==='Balius'?[{id:'balius',componentId:stages[0]?.componentId||''}]:[{id:'sustainer',componentId:stages[0]?.componentId||''},{id:'booster',componentId:stages[1]?.componentId||''}];
    state.mission.video.streams=preset==='Balius'?[{id:'balius.digital',vehicleId:'balius',kind:'digital',receivers:['gs1','gs2','gs3','gs4']},{id:'balius.analog',vehicleId:'balius',kind:'analog',receivers:['gs1','gs2','gs3','gs4']}]:[{id:'sustainer.digital',vehicleId:'sustainer',kind:'digital',receivers:['gs1','gs2','gs3','gs4']},{id:'sustainer.analog',vehicleId:'sustainer',kind:'analog',receivers:['gs1','gs2','gs3']},{id:'booster.analog',vehicleId:'booster',kind:'analog',receivers:['gs4']}];
    for(const pointer of state.mission.pointers)pointer.targetVehicleId=preset==='Iris'?(pointer.siteId==='gs4'?'booster':'sustainer'):'balius';
    for(const station of state.mission.stationBindings)station.targetVehicleId=preset==='Iris'?(station.siteId==='gs4'?'booster':'sustainer'):'balius';
    changed();document.dispatchEvent(new Event('mission-render'));
  }else if(event.detail.startsWith('video.'))slate();
});
