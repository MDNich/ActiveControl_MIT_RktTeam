import {state,$,esc,get,set,changed,action,request,draftAction,useMission,status} from './api.js';
export const label = key => key.replace(/([a-z])([A-Z])/g,'$1 $2').replace(/_/g,' ').replace(/^./,s=>s.toUpperCase());
export function field(path,title,{choices,type,scale=1,offset=0,min,max,step='any',note='',readonly=false,stringNumber=false}={}) {
  const value=get(path), id='field-'+path.replaceAll('.','-');
  type??=typeof value==='boolean'?'checkbox':typeof value==='number'?'number':'text';
  const required=type==='number'&&!path.endsWith('.heightM')&&!path.endsWith('.ejectionDelayS');
  const attrs=`${required?'required':''} id="${esc(id)}" data-field="${esc(path)}" data-scale="${scale}" data-offset="${offset}" ${stringNumber?'data-string-number="true"':''} ${readonly?'disabled':''}`;
  let input;
  if(choices) input=`<select ${attrs}>${choices.map(c=>{const [v,n]=Array.isArray(c)?c:[c,label(c)];return `<option value="${esc(v)}" ${String(v)===String(value)?'selected':''}>${esc(n)}</option>`;}).join('')}</select>`;
  else if(type==='json')input=`<textarea ${attrs} data-json="true" rows="3">${esc(JSON.stringify(value))}</textarea>`;
  else if(type==='textarea')input=`<textarea ${attrs} rows="3">${esc(value)}</textarea>`;
  else input=`<input ${attrs} type="${type}" ${type==='checkbox'?(value?'checked':''):`value="${esc(type==='number'&&value!=null?Number(value)*scale+offset:value??'')}"`} ${min!==undefined?`min="${min}"`:''} ${max!==undefined?`max="${max}"`:''} step="${step}">`;
  return `<label for="${esc(id)}">${esc(title)}${input}${note?`<span class="field-note">${esc(note)}</span>`:''}</label>`;
}
export function section(title,fields,note='') {return `<div class="form-section"><h2>${esc(title)}</h2>${note?`<p class="hint">${esc(note)}</p>`:''}<div class="field-grid">${fields.join('')}</div></div>`;}
export function attachFields(root=document) {
  root.querySelectorAll('[data-field]').forEach(el=>el.onchange=()=>{
    if(el.type==='number' && !el.checkValidity()){el.reportValidity();return;}
    try {
      el.setCustomValidity('');
      let value=el.type==='checkbox'?el.checked:el.type==='number'?(el.value===''?null:(Number(el.value)-Number(el.dataset.offset))/Number(el.dataset.scale)):el.value;
      if(el.dataset.field.endsWith('.timeUs')&&value!==null)value=Math.round(value);
      if(el.tagName==='SELECT'&&el.dataset.field.endsWith('.opcode'))value=Number(value);
      if(el.dataset.stringNumber&&value!==null)value=String(value);
      if(el.dataset.json)value=JSON.parse(value);
      set(el.dataset.field,value); document.dispatchEvent(new CustomEvent('field-changed',{detail:el.dataset.field}));
    } catch(error){el.setCustomValidity(error.message);el.reportValidity();status('Value not applied: '+error.message);}
  });
}
const deg=180/Math.PI;
export function renderFlight() {
  const editor=get('flight.editor');
  $('rocket-name').textContent=editor?`${get('flight.filename')} · ${get('flight.simulationName')}`:'Choose a rocket to begin.';
  if(!editor){$('flight-fields').innerHTML='<div class="information">Load a saved flight to configure motors, stages, recovery, weather, the solver, and flight computer settings.</div>';return;}
  const p='flight.editor.options.';
  let html=section('Launch guide',[
    field(p+'guideLengthM','Guide length · m',{min:0,type:'number'}),field(p+'guideAngleRad','Tilt from vertical · °',{scale:deg,min:-90,max:90,type:'number'}),
    field(p+'manualBearingRad','Manual bearing · ° true north',{scale:deg,min:0,max:360,type:'number'}),field(p+'intoWind','Point launch guide into wind'),field(p+'initialRollRateRadS','Initial roll rate · °/s',{scale:deg,type:'number'})
  ],'The launch location and elevation come from Site & stations. The saved manual bearing is retained when launching into wind.');
  html+=section('Wind',[
    field(p+'windModel','Model',{choices:[['AVERAGE','Average wind'],['MULTI_LEVEL','Altitude profile']]}),field(p+'averageWind.speedMS','Average speed · m/s',{type:'number',min:0}),
    field(p+'averageWind.directionRad','Direction · °',{type:'number',scale:deg,min:0,max:360}),field(p+'averageWind.sigmaMS','Speed standard deviation · m/s',{type:'number',min:0}),field(p+'windAltitudeReference','Profile altitude reference',{choices:[['MSL','Mean sea level'],['AGL','Above launch']]})
  ]);
  html+='<div class="form-section"><h2>Wind profile</h2><div class="rows">'+editor.options.windLevels.map((v,i)=>`<div class="row-card"><div class="field-grid">${field(p+`windLevels.${i}.altitudeM`,'Altitude · m',{type:'number'})}${field(p+`windLevels.${i}.speedMS`,'Speed · m/s',{type:'number',min:0})}${field(p+`windLevels.${i}.directionRad`,'Direction · °',{type:'number',scale:deg,min:0,max:360})}${field(p+`windLevels.${i}.sigmaMS`,'Standard deviation · m/s',{type:'number',min:0})}</div><button data-remove-wind="${i}">Remove level</button></div>`).join('')+'</div><button id="add-wind">Add wind level</button></div>';
  html+=section('Atmosphere & flight solver',[
    field(p+'isa','Use standard atmosphere'),field(p+'temperatureK','Launch temperature · °C',{type:'number',offset:-273.15}),field(p+'pressurePa','Launch pressure · hPa',{type:'number',scale:.01,min:.001}),
    field(p+'geodeticMethod','Earth model',{choices:[['FLAT','Flat Earth'],['SPHERICAL','Spherical Earth'],['WGS84','WGS84 ellipsoid']]}),field(p+'timeStepS','Maximum time step · s',{type:'number',min:.000001,max:10}),field(p+'maximumTimeS','Maximum flight duration · s',{type:'number',min:.001}),field(p+'maximumStepAngleRad','Maximum angle per step · °',{type:'number',scale:deg,min:1,max:20}),field(p+'randomSeed','Flight random seed',{type:'number',step:1,min:-2147483648,max:2147483647})
  ]);
  for(const [group,title,eventChoices] of [['stages','Stages & separation',state.capabilities.choices.separation],['motors','Motors & ignition',state.capabilities.choices.ignition],['recovery','Recovery devices',state.capabilities.choices.deployment]]) {
    html+=`<div class="form-section"><h2>${title}</h2><div class="rows">`+editor[group].map((v,i)=>{
      const path=`flight.editor.${group}.${i}.`;let fields=[];
      if(group==='stages')fields.push(field(path+'active','Active in this flight'));
      if(group==='motors')fields.push(`<label>Motor curve<div class="path-value">${esc(v.motorName)}</div><button data-motor="${i}">Choose motor</button></label>`,field(path+'ignitionOverride','Custom ignition'),field(path+'ejectionDelayS','Ejection delay · s',{type:'number',min:0,note:'Blank means plugged / no motor ejection.'}));
      fields.push(field(path+'event',group==='recovery'?'Deploy on':group==='motors'?'Ignition event':'Separation event',{choices:eventChoices}),field(path+'delayS','Delay · s',{type:'number',min:0}));
      if(group!=='motors')fields.push(field(path+'altitudeM','Event altitude · m',{type:'number',min:group==='stages'?-1000:0}));
      return `<div class="row-card"><h3>${esc(v.name)}</h3><div class="field-grid">${fields.join('')}</div></div>`;
    }).join('')+'</div></div>';
  }
  html+='<div class="form-section"><h2>Flight computer & extensions</h2><button id="choose-fc">Choose flight computer .fc</button><p class="hint">Select a custom FC in Telemetry to run it. Standard telemetry runs the ordinary rocket flight.</p>';
  editor.extensions.forEach((extension,i)=>{
    html+=`<details><summary>${esc(extension.name)} · complete settings</summary><div class="field-grid">`;
    for(const [key,value] of Object.entries(extension.config))html+=field(`flight.editor.extensions.${i}.config.${key.replaceAll('~','~0').replaceAll('.','~1')}`,label(key),{readonly:['designFile','designId','computer'].includes(key),type:Array.isArray(value)?'json':typeof value==='string'&&value.length>100?'textarea':undefined});
    html+='</div></details>';
  });
  html+='</div><div class="form-section"><h2>Vehicle bindings</h2><p class="hint">Bind each logical vehicle to a rocket stage. A multi-stage rehearsal needs explicit stage assignments.</p><div class="rows">'+state.mission.vehicleBindings.map((v,i)=>`<div class="row-card"><h3>${esc(v.id)}</h3>${field(`vehicleBindings.${i}.componentId`,'Rocket stage',{choices:[['','Choose stage'],...editor.stages.map(s=>[s.componentId,s.name])]})}</div>`).join('')+'</div></div>';
  $('flight-fields').innerHTML=html;attachFields($('flight-fields'));
  $('add-wind').onclick=()=>{const levels=editor.options.windLevels;levels.push({altitudeM:levels.length?Math.max(...levels.map(x=>x.altitudeM))+1000:0,speedMS:0,directionRad:0,sigmaMS:0});changed();renderFlight();};
  document.querySelectorAll('[data-remove-wind]').forEach(b=>b.onclick=()=>{editor.options.windLevels.splice(Number(b.dataset.removeWind),1);changed();renderFlight();});
  $('choose-fc').onclick=()=>action(async()=>{const result=await draftAction('fc');if(!result.cancelled){useMission(result);status('Flight computer frozen with this draft. Choose Custom FC in Telemetry to execute it.');}});
  document.querySelectorAll('[data-motor]').forEach(b=>b.onclick=()=>motorDialog(Number(b.dataset.motor)));
}
function motorDialog(index) {
  let dialog=document.getElementById('motor-dialog');if(dialog)dialog.remove();dialog=document.createElement('dialog');dialog.id='motor-dialog';
  dialog.innerHTML='<h2>Choose motor curve</h2><label>Search designation or manufacturer<input id="motor-query"></label><div class="actions"><button id="motor-search">Search</button><button id="motor-none">No motor</button><button id="motor-close">Close</button></div><div id="motor-results" class="rows"></div>';document.body.append(dialog);dialog.showModal();
  $('motor-close').onclick=()=>dialog.close();$('motor-none').onclick=()=>{set(`flight.editor.motors.${index}.motorDigest`,'');set(`flight.editor.motors.${index}.motorName`,'No motor');dialog.close();renderFlight();};
  $('motor-search').onclick=async()=>{try{const data=await request('motors',{query:$('motor-query').value});$('motor-results').innerHTML=data.motors.map((m,i)=>`<button data-choice="${i}">${esc(m.name)} · ${(m.diameterM*1000).toFixed(1)} × ${(m.lengthM*1000).toFixed(1)} mm</button>`).join('');$('motor-results').querySelectorAll('button').forEach(b=>b.onclick=()=>{const m=data.motors[Number(b.dataset.choice)];set(`flight.editor.motors.${index}.motorDigest`,m.digest);set(`flight.editor.motors.${index}.motorName`,m.name);dialog.close();renderFlight();});}catch(e){status(e.message);}};
}
export function renderEnsemble() {
  if(!get('flight.editor'))return '<div class="information">Choose a rocket and flight before configuring an ensemble.</div>';
  const p='flight.editor.options.ensemble.';state.mission.execution.applicationMember??=0;
  const fields=[field(p+'enabled','Flight ensemble',{choices:[['false','Single flight'],['true','Ensemble']]}),field(p+'runs','Flight members',{type:'number',stringNumber:true,min:2,max:10000,step:1}),field(p+'threads','Parallel workers',{type:'number',stringNumber:true,min:1,max:256,step:1}),field(p+'seed','Ensemble seed',{type:'number',stringNumber:true,step:1}),field(p+'motorsigma','Thrust noise σ · N',{type:'number',stringNumber:true,min:0}),field(p+'noiseinterval','Thrust noise interval · s',{type:'number',stringNumber:true,min:.005,max:10}),field(p+'temperaturesigma','Temperature σ · K',{type:'number',stringNumber:true,min:0,max:50}),field(p+'pressuresigma','Pressure σ · %',{type:'number',stringNumber:true,scale:100,min:0,max:30}),field(p+'windsigma','Wind σ · m/s',{type:'number',stringNumber:true,min:0}),field(p+'winddirectionsigma','Wind direction σ · °',{type:'number',stringNumber:true,scale:deg,min:0,max:180})];
  for(const [key,title] of [['masssigma','Mass'],['dragsigma','Drag'],['rollforcesigma','Roll forcing'],['rolldampingsigma','Roll damping']]) fields.push(field(p+key,title+' σ · %',{type:'number',stringNumber:true,scale:100,min:0,max:100}));
  fields.push(field('execution.applicationMember','Application member · zero-based index',{type:'number',min:0,max:Number(get(p+'runs'))-1,step:1,note:'Only this member opens connected applications. Other members use headless station policies.'}));
  return section('Single flight or ensemble',fields,'Flight uncertainty uses the existing OpenRocket ensemble models and seeds. Every member receives independent mission station state.');
}

export function outages(path,values){return `<label>Outage windows · seconds<input data-outages="${path}" value="${esc(values.map(v=>`${Number(v.startUs)/1e6}-${Number(v.endUs)/1e6}`).join(', '))}" placeholder="12-15, 40-42"><span class="field-note">Start-end pairs, separated by commas.</span></label>`;}
export function bindOutages(root){root.querySelectorAll('[data-outages]').forEach(el=>el.onchange=()=>{try{el.setCustomValidity('');const windows=el.value.trim()?el.value.split(',').map((part,i)=>{const match=part.trim().match(/^(-?\d+(?:\.\d+)?)\s*[-–]\s*(-?\d+(?:\.\d+)?)$/);if(!match||Number(match[2])<=Number(match[1]))throw Error('Use increasing start-end time pairs, such as 12-15, 40-42.');return {id:'outage-'+(i+1),startUs:String(Math.round(Number(match[1])*1e6)),endUs:String(Math.round(Number(match[2])*1e6))};}):[];set(el.dataset.outages,windows);}catch(error){el.setCustomValidity(error.message);el.reportValidity();status(error.message);}});}
