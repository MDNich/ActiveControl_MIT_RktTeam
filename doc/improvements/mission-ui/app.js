/* Configuration prototype. No simulator, process launch, device or video service is invoked. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const esc = x => String(x ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const initial = {
    format:'openrocket.mission-configuration-preview', version:1,
    title:'URRG mission rehearsal',runId:'MS-024',rocket:'20267.demonstrator.iteration4.ork',pointerCount:4,receiverSites:[],
    sites:[
      {id:'launch-site',name:'Launch site',symbol:'L',lat:42.70416776971639,lon:-77.19029501648315,alt:null,datum:'MSL',coordinateInput:{format:'mgrs',value:'18TUN2061530290',reference:'grid-point',precisionM:1,source:'telemetry-ui-urrg'}},
      {id:'launch',name:'Base station',symbol:'B',lat:42.7031920173103,lon:-77.18999209440938,alt:null,datum:'MSL',coordinateInput:{format:'mgrs',value:'18TUN2063730181',reference:'grid-point',precisionM:1,source:'telemetry-ui-urrg'}},
      {id:'gs1',name:'Away 1 · Pointer 1',symbol:'1',lat:42.70278230997951,lon:-77.17605911889694,alt:null,datum:'MSL',coordinateInput:{format:'mgrs',value:'18TUN2177730106',reference:'grid-point',precisionM:1,source:'telemetry-ui-urrg'},height:2,azRate:90,elRate:60,az:0,el:0,policy:'Geodetic'},
      {id:'gs2',name:'Away 2 · Pointer 2',symbol:'2',lat:42.731221129355355,lon:-77.16317739743833,alt:null,datum:'MSL',coordinateInput:{format:'mgrs',value:'18TUN2291333237',reference:'grid-point',precisionM:1,source:'telemetry-ui-urrg'},height:2,azRate:90,elRate:60,az:0,el:0,policy:'Geodetic'},
      {id:'gs3',name:'Away 3 · Pointer 3',symbol:'3',lat:42.71135721362275,lon:-77.19622600691332,alt:null,datum:'MSL',coordinateInput:{format:'mgrs',value:'18TUN2015031101',reference:'grid-point',precisionM:1,source:'telemetry-ui-urrg'},height:2,azRate:90,elRate:60,az:0,el:0,policy:'Geodetic'},
      {id:'gs4',name:'Away 4 · Pointer 4',symbol:'4',lat:42.69911925972465,lon:-77.16598116908455,alt:null,datum:'MSL',coordinateInput:{format:'mgrs',value:'18TUN2259229678',reference:'grid-point',precisionM:1,source:'telemetry-ui-urrg'},height:2,azRate:90,elRate:60,az:0,el:0,policy:'Geodetic'}
    ],
    flight:{configuration:'Demonstrator · nominal',sourceSimulation:'Example settings',stages:'All active stages',motors:'Use selected flight configuration',ignition:'Use saved ignition events and delays',separation:'Use saved stage events and delays',recovery:'OpenRocket recovery rules',recoveryEvents:'Use saved device events, delays and deployment altitudes',rail:5,tilt:5,bearing:0,intoWind:false,roll:0,windModel:'Average',windSpeed:4,windDirection:270,turbulence:10,windSigma:.4,windProfile:'0, 4, 270\n1000, 6, 280\n3000, 8, 290',windAltitude:'MSL',atmosphere:'ISA',temperature:15,pressure:1013.25,geodetic:'Spherical',timeStep:.05,maxTime:600,maxAngle:3,seed:1,extensions:'Preserve selected simulation extensions'},
    telemetry:{mode:'standard',rate:20,fcPath:'',definition:'',fallback:'legacy',loss:2,delay:30,jitter:5,bitError:0,gpsHorizontal:2,gpsVertical:3,barometer:.5,seed:101,outages:'',overrides:{}},
    video:{preset:'Balius',digital:1,analog:1,streams:[],fps:30,width:960,height:540,delay:100,loss:0,noise:0,seed:201},
    connections:{path:'/Users/mdn/Developer/RT_Telemetry_UI',launch:'Python project',python:'/Users/mdn/Developer/RT_Telemetry_UI/.venv/bin/python',transport:'Software serial',stations:{}},
    run:{mode:'Deterministic mission',type:'Single flight',pace:'As fast as possible',count:100,threads:4,seed:1,motorSigma:0,noiseInterval:.05,temperatureSigma:0,pressureSigma:0,windSigma:0,windDirectionSigma:0,massSigma:0,dragSigma:0,rollForceSigma:0,rollDampingSigma:0,output:'./mission-results',filename:'urrg-mission-rehearsal.rktmission',fcLogs:true,raw:true,pointerLogs:true,video:true}
  };
  let state=structuredClone(initial), page='site', selectedSite='launch-site', coordMode='mgrs', placing=false, selectedStream='digital-1', map, layer, markers=[];
  const pages=['site','flight','telemetry','video','connections','run'];
  const titles=['Site & stations','Rocket & flight','Telemetry','Video','Connections','Run & results'];
  const descriptions={site:'URRG launch and station defaults from Telemetry UI · elevations still need to be entered.',flight:'Flight configuration and physics remain independent of the ground network.',telemetry:'One source message; separate receiver observations and link conditions.',video:'Vehicle presets are starting points. Every stream has its own identity.',connections:'A separate simulated endpoint for each station role.',run:'Reusable configuration in; traceable mission bundle out.'};
  function get(path){return path.split('.').reduce((o,k)=>o?.[k],state);}
  function set(path,value){const keys=path.split('.');const last=keys.pop();const target=keys.reduce((o,k)=>o[k],state);target[last]=value;}
  function status(text){$('status').textContent=text;}
  function activeSites(){return state.sites.filter((s,i)=>i<state.pointerCount+2||state.receiverSites.includes(s.id));}
  function stations(){return activeSites().slice(1);}
  function site(){return state.sites.find(s=>s.id===selectedSite);}
  function siteName(s){return s.id.startsWith('gs')&&Number(s.id.slice(2))>state.pointerCount?'Away '+s.id.slice(2)+' · Receiver':s.name;}
  function field(path,label,type='number',extra={}){
    const id='f-'+path.replaceAll('.','-');
    let input;
    if(Array.isArray(type)) input=`<select id="${id}" data-bind="${path}">${type.map(v=>`<option>${esc(v)}</option>`).join('')}</select>`;
    else if(type==='checkbox') input=`<input id="${id}" type="checkbox" data-bind="${path}">`;
    else input=`<input id="${id}" type="${type}" data-bind="${path}" ${type==='number'?'step="any"':''} ${Object.entries(extra).map(([k,v])=>`${k}="${esc(v)}"`).join(' ')}>`;
    return `<label for="${id}">${esc(label)}${input}</label>`;
  }
  function group(title,fields){return `<div class="form-section"><h2>${title}</h2><div class="three-col">${fields.join('')}</div></div>`;}
  function bind(){
    document.querySelectorAll('[data-bind]').forEach(el=>{
      const key=el.dataset.bind;
      if(el.type==='checkbox')el.checked=Boolean(get(key));else el.value=get(key)??'';
      el.onchange=()=>{
        if(el.type==='number' && (!el.checkValidity() || el.value==='')){el.reportValidity();return;}
        set(key,el.type==='checkbox'?el.checked:el.type==='number'?Number(el.value):el.value);
        updateDerived();status('Configuration changed · save to keep a reusable copy.');
      };
    });
  }
  function buildForms(){
    $('flight-fields').innerHTML=
      group('Motors, stages & recovery',[field('flight.stages','Active stages','text'),field('flight.motors','Motor assignments','text'),field('flight.ignition','Ignition events & delays','text'),field('flight.separation','Stage separation','text'),field('flight.recovery','Recovery authority',['OpenRocket recovery rules','Flight computer mappings']),field('flight.recoveryEvents','Recovery device settings','text')])+
      group('Launch guide',[field('flight.rail','Guide length m','number',{min:0}),field('flight.tilt','Tilt from vertical °'),field('flight.bearing','Bearing ° true north','number',{min:0,max:360}),field('flight.intoWind','Launch into wind','checkbox'),field('flight.roll','Initial roll rate °/s')])+
      group('Wind',[field('flight.windModel','Wind model',['Average','Multi-level']),field('flight.windSpeed','Average speed m/s','number',{min:0}),field('flight.windDirection','Direction °','number',{min:0,max:360}),field('flight.turbulence','Turbulence intensity %','number',{min:0}),field('flight.windSigma','Speed standard deviation m/s','number',{min:0}),field('flight.windAltitude','Wind-level altitude reference',['MSL','Above launch'])])+
      '<div class="form-section" id="wind-levels" hidden><h2>Wind levels</h2><label>Altitude m, speed m/s, direction °<textarea data-bind="flight.windProfile" rows="4" style="width:100%;margin-top:7px"></textarea></label></div>'+
      group('Atmosphere',[field('flight.atmosphere','Model',['ISA','Custom launch conditions']),field('flight.temperature','Launch temperature °C'),field('flight.pressure','Launch pressure hPa','number',{min:1})])+
      group('Flight solver & extensions',[field('flight.geodetic','Geodetic model',['Flat','Spherical','WGS84']),field('flight.timeStep','Time step s','number',{min:.001}),field('flight.maxTime','Maximum flight time s','number',{min:1}),field('flight.maxAngle','Maximum step angle °','number',{min:.01}),field('flight.seed','Flight random seed','number',{step:1}),field('flight.extensions','Enabled extensions & settings','text')]);
    $('telemetry-fields').innerHTML='<div class="three-col">'+[
      field('telemetry.loss','Packet loss %','number',{min:0,max:100}),field('telemetry.delay','Delivery delay ms','number',{min:0}),field('telemetry.jitter','Delay jitter σ ms','number',{min:0}),field('telemetry.bitError','Bit error probability','number',{min:0,max:1}),field('telemetry.gpsHorizontal','GPS horizontal noise σ m','number',{min:0}),field('telemetry.gpsVertical','GPS vertical noise σ m','number',{min:0}),field('telemetry.barometer','Barometric altitude noise σ m','number',{min:0}),field('telemetry.seed','Link/noise seed','number',{step:1}),field('telemetry.outages','Outage windows s · e.g. 12–15, 40–42','text')].join('')+'</div>';
    $('video-fields').innerHTML='<div class="three-col">'+[field('video.fps','Frames per second','number',{min:1,max:60,step:1}),field('video.width','Image width px','number',{min:160,max:3840,step:2}),field('video.height','Image height px','number',{min:90,max:2160,step:2}),field('video.delay','Playback delay ms','number',{min:0}),field('video.loss','Dropout %','number',{min:0,max:100}),field('video.noise','Analog picture noise %','number',{min:0,max:100}),field('video.seed','Video impairment seed','number',{step:1})].join('')+'</div>';
    $('ensemble-fields').innerHTML='<div class="three-col">'+[field('run.count','Flights','number',{min:2,max:10000,step:1}),field('run.threads','Parallel workers','number',{min:1,max:256,step:1}),field('run.seed','Ensemble seed','number',{step:1}),field('run.motorSigma','Thrust noise σ N','number',{min:0}),field('run.noiseInterval','Thrust noise interval s','number',{min:.005,max:10}),field('run.temperatureSigma','Temperature σ K','number',{min:0,max:50}),field('run.pressureSigma','Pressure σ %','number',{min:0,max:30}),field('run.windSigma','East/north wind σ m/s','number',{min:0}),field('run.windDirectionSigma','Wind direction σ °','number',{min:0,max:180}),field('run.massSigma','Total mass σ %','number',{min:0,max:100}),field('run.dragSigma','Overall drag σ %','number',{min:0,max:100}),field('run.rollForceSigma','Roll forcing σ %','number',{min:0,max:100}),field('run.rollDampingSigma','Roll damping σ %','number',{min:0,max:100})].join('')+'</div>';
  }
  function showPage(name){
    page=name;document.querySelectorAll('.page').forEach(el=>el.hidden=el.id!=='page-'+name);
    document.querySelectorAll('nav button').forEach(el=>{el.classList.toggle('active',el.dataset.page===name);el.setAttribute('aria-current',el.dataset.page===name?'page':'false');});
    $('page-description').textContent=descriptions[name];
    const i=pages.indexOf(name);$('next-page').textContent=i<5?'Next: '+titles[i+1]+' →':'Back to site & stations';
    if(name==='site' && map)setTimeout(()=>map.invalidateSize(),40);
    updateDerived();
  }
  function pointValid(s){return Number.isFinite(s.lat)&&Number.isFinite(s.lon)&&Math.abs(s.lat)<=90&&Math.abs(s.lon)<=180;}
  function renderSites(){
    if(!activeSites().some(s=>s.id===selectedSite))selectedSite='launch-site';
    $('pointer-count').value=state.pointerCount;
    $('site-list').innerHTML=activeSites().map(s=>`<button class="site-choice ${s.id===selectedSite?'active':''}" data-site="${s.id}" aria-pressed="${s.id===selectedSite}"><span class="site-symbol ${s.id==='launch-site'?'launch':s.id==='launch'?'base':''}">${s.symbol}</span><span><strong>${siteName(s)}</strong><small>${pointValid(s)?s.lat.toFixed(5)+', '+s.lon.toFixed(5):'Needs a location'}</small></span></button>`).join('');
    $('site-list').querySelectorAll('button').forEach(b=>b.onclick=()=>{selectedSite=b.dataset.site;placing=false;renderSites();});
    const s=site();$('site-title').textContent=siteName(s);$('latitude').value=s.lat??'';$('longitude').value=s.lon??'';$('elevation').value=s.alt??'';$('datum').value=s.datum;
    $('mgrs').value=s.coordinateInput?.format==='mgrs'?s.coordinateInput.value:pointValid(s)&&s.lat>=-80&&s.lat<=84&&window.mgrs?window.mgrs.forward([s.lon,s.lat],5):'';
    $('mgrs-precision').textContent=s.coordinateInput?.format==='mgrs'?(s.coordinateInput.reference==='grid-point'?'Telemetry UI grid reference point':'Grid cell center')+' · '+s.coordinateInput.precisionM+' m grid resolution.':'WGS84 · new MGRS entries use the cell center.';
    $('coordinate-error').textContent='';$('place-site').classList.toggle('active',placing);$('place-site').textContent=placing?'Click the map to place…':'Place on map';
    $('pointer-details').hidden=!s.id.startsWith('gs')||Number(s.id.slice(2))>state.pointerCount;
    if(s.id.startsWith('gs')){
      $('pointer-settings').innerHTML='<div class="two-col">'+[['height','Mount height m'],['azRate','Azimuth rate °/s'],['elRate','Elevation rate °/s'],['az','Initial azimuth °'],['el','Initial elevation °']].map(([key,label])=>`<label>${label}<input type="number" step="any" data-pointer="${key}" value="${s[key]}"></label>`).join('')+'</div><p class="hint">Illustrative mount rates. Validate against your hardware.</p>';
      $('pointer-settings').querySelectorAll('input').forEach(el=>el.onchange=()=>{if(el.value===''||!Number.isFinite(Number(el.value)))return;site()[el.dataset.pointer]=Number(el.value);status('Pointer model updated.');});
    }
    const launch=state.sites[0];$('site-distance').textContent=s.id==='launch-site'?'Elevation is entered manually; imagery does not set it.':pointValid(s)&&pointValid(launch)&&map?(map.distance([launch.lat,launch.lon],[s.lat,s.lon])/1000).toFixed(2)+' km from launch · elevation entered manually':'Place this station to calculate distance.';
    drawMarkers();renderReceivers();renderConnections();updateDerived();
  }
  function coordinateMode(mode){coordMode=mode;$('latlon-fields').hidden=mode!=='latlon';$('mgrs-fields').hidden=mode!=='mgrs';['latlon','mgrs'].forEach(k=>{$('mode-'+k).classList.toggle('active',k===mode);$('mode-'+k).setAttribute('aria-pressed',k===mode);});}
  function applyCoordinates(){
    try{
      let lat,lon,coordinateInput;
      if(coordMode==='mgrs'){
        if(!window.mgrs)throw Error('Coordinate converter is unavailable. Use latitude/longitude.');
        const v=$('mgrs').value.toUpperCase().replace(/\s/g,'');
        if(!/^\d{1,2}[C-HJ-NP-X][A-HJ-NP-Z]{2}(?:\d{2}){0,5}$/.test(v))throw Error('Enter a valid UTM MGRS reference with equal easting/northing precision.');
        const previous=site().coordinateInput;
        if(previous?.format==='mgrs'&&previous.value===v){({lat,lon}=site());coordinateInput=structuredClone(previous);}
        else{[lon,lat]=window.mgrs.toPoint(v);const digits=v.match(/\d+$/)?.[0].length??0;coordinateInput={format:'mgrs',value:v,reference:'cell-center',precisionM:Math.pow(10,5-digits/2),source:'manual'};}
      }else{if(!$('latitude').value.trim()||!$('longitude').value.trim())throw Error('Enter both latitude and longitude.');lat=Number($('latitude').value);lon=Number($('longitude').value);coordinateInput=lat===site().lat&&lon===site().lon?structuredClone(site().coordinateInput??{format:'latlon',source:'manual'}):{format:'latlon',source:'manual'};}
      if(!pointValid({lat,lon}))throw Error('Latitude must be −90…90 and longitude −180…180.');
      const altitude=$('elevation').value.trim()===''?null:Number($('elevation').value);
      if(altitude!==null&&!Number.isFinite(altitude))throw Error('Enter a valid elevation or leave it unset.');
      Object.assign(site(),{lat,lon,alt:altitude,datum:$('datum').value,coordinateInput});renderSites();if(map)map.panTo([lat,lon]);status('Coordinates applied · '+siteName(site())+'.');
    }catch(e){$('coordinate-error').textContent=e.message;}
  }
  function drawMarkers(){
    if(!map)return;markers.forEach(m=>map.removeLayer(m));markers=[];
    activeSites().filter(pointValid).forEach(s=>{
      const type=s.id==='launch-site'?'launch':s.id==='launch'?'base':'';
      const icon=L.divIcon({html:`<div class="map-pin ${type} ${s.id===selectedSite?'selected':''}">${s.symbol}</div>`,className:'',iconSize:[30,30],iconAnchor:s.id==='launch'?[48,15]:[15,15]});
      const m=L.marker([s.lat,s.lon],{icon,draggable:true,title:siteName(s),alt:siteName(s),zIndexOffset:s.id===selectedSite?1000:0}).addTo(map).bindTooltip(siteName(s),{direction:'top',offset:[s.id==='launch'?-33:0,-19]});
      m.on('click',()=>{selectedSite=s.id;placing=false;renderSites();});
      m.on('dragend',()=>{const p=m.getLatLng();s.lat=p.lat;s.lon=p.lng;s.coordinateInput={format:'latlon',source:'map'};selectedSite=s.id;renderSites();status(siteName(s)+' moved · elevation unchanged.');});markers.push(m);
    });
  }
  function setLayer(){
    if(!map)return;if(layer)map.removeLayer(layer);
    const type=$('map-layer').value;
    const sources={
      hybrid:['https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryTopo/MapServer/tile/{z}/{y}/{x}',{attribution:'USGS The National Map: Orthoimagery and US Topo',maxNativeZoom:16,maxZoom:19}],
      satellite:['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{attribution:'Sources: Esri, Maxar, Earthstar Geographics, and the GIS User Community',maxZoom:19}],
      street:['https://tile.openstreetmap.org/{z}/{x}/{y}.png',{attribution:'© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap contributors</a>',maxZoom:19}]
    };
    $('map-status').textContent='Loading imagery…';let failures=0,loaded=0;
    layer=L.tileLayer(...sources[type]).addTo(map);
    layer.on('tileload',()=>{loaded++;$('map-status').textContent=failures?'Some map tiles unavailable':'Imagery loaded';});
    layer.on('tileerror',()=>{failures++;$('map-status').textContent=loaded?'Some map tiles unavailable':'Map unavailable · coordinates still editable';});
  }
  function initMap(){
    if(!window.L){$('map-status').textContent='Map library unavailable · coordinates still editable';return;}
    map=L.map('map',{scrollWheelZoom:false}).fitBounds(activeSites().filter(pointValid).map(s=>[s.lat,s.lon]),{padding:[50,50],maxZoom:15});L.control.scale({imperial:false}).addTo(map);setLayer();
    new ResizeObserver(()=>map.invalidateSize()).observe($('map'));
    map.on('click',e=>{if(!placing)return;Object.assign(site(),{lat:e.latlng.lat,lon:e.latlng.lng,coordinateInput:{format:'latlon',source:'map'}});placing=false;renderSites();status(siteName(site())+' placed · set its elevation separately.');});
    drawMarkers();
  }
  function renderReceivers(){
    $('receiver-overrides').innerHTML=stations().map(s=>{
      const o=state.telemetry.overrides[s.id]??={enabled:false,loss:state.telemetry.loss,delay:state.telemetry.delay};
      return `<div class="receiver-row"><div class="row-title">${s.id==='launch'?'Base station':siteName(s)}<small>${s.id} · independent reception</small></div><label><input type="checkbox" data-override="${s.id}:enabled" ${o.enabled?'checked':''}> Override defaults</label><label>Loss %<input type="number" min="0" max="100" data-override="${s.id}:loss" value="${o.loss}" ${o.enabled?'':'disabled'}></label><label>Delay ms<input type="number" min="0" data-override="${s.id}:delay" value="${o.delay}" ${o.enabled?'':'disabled'}></label></div>`;
    }).join('');
    $('receiver-overrides').querySelectorAll('input').forEach(el=>el.onchange=()=>{if(!el.checkValidity()){el.reportValidity();return;}const [id,key]=el.dataset.override.split(':');state.telemetry.overrides[id][key]=el.type==='checkbox'?el.checked:Number(el.value);renderReceivers();status('Receiver override updated.');});
  }
  function createStreams(reset=false){
    const old=state.video.streams;const next=[];
    for(const kind of ['digital','analog'])for(let i=1;i<=state.video[kind];i++){
      const id=kind+'-'+i;let stream=!reset&&old.find(s=>s.id===id);
      if(!stream)stream={id,kind,number:i,vehicle:state.video.preset==='Iris'?(kind==='analog'&&i===2?'Booster':'Sustainer'):'Balius',receivers:state.video.preset==='Iris'?(kind==='analog'?(i===2?['gs4']:['gs1','gs2','gs3']):['gs1','gs2','gs3','gs4']):stations().filter(s=>s.id!=='launch').map(s=>s.id)};
      next.push(stream);
    }
    state.video.streams=next;if(!next.some(s=>s.id===selectedStream))selectedStream=next[0]?.id??'';
  }
  function renderStreams(){
    $('video-preset').value=state.video.preset;$('digital-count').value=state.video.digital;$('analog-count').value=state.video.analog;
    $('video-support').textContent=state.video.preset==='Iris'?'Iris preset: Sustainer digital + Sustainer analog + Booster analog. Current UI uses fixed slots.':state.video.preset==='Balius'?'Balius preset: one digital + one analog. Custom counts require a stream-registry extension in the Telemetry UI.':'Custom stream counts are part of the proposed integration; the current UI has fixed vehicle-specific slots.';
    $('stream-list').innerHTML=state.video.streams.length?state.video.streams.map(s=>`<div class="stream-card ${s.id===selectedStream?'selected':''}"><div class="stream-heading"><strong>${s.kind==='digital'?'Digital':'Analog'} ${s.number}</strong><button data-preview="${s.id}" aria-pressed="${s.id===selectedStream}">Preview</button></div><label>Vehicle<select data-stream="${s.id}:vehicle">${['Balius','Sustainer','Booster','Unassigned'].map(v=>`<option ${s.vehicle===v?'selected':''}>${v}</option>`).join('')}</select></label><div class="checks">${['launch','gs1','gs2','gs3','gs4'].map(v=>`<label><input type="checkbox" data-receive="${s.id}:${v}" ${s.receivers.includes(v)?'checked':''}>${v==='launch'?'Base':'Away '+v.slice(2)}${stations().some(x=>x.id===v&&pointValid(x))?'':'*'}</label>`).join('')}</div><p class="hint">Receiving stations · * location not configured</p></div>`).join(''):'<div class="information">No video streams configured.</div>';
    $('stream-list').querySelectorAll('[data-preview]').forEach(b=>b.onclick=()=>{selectedStream=b.dataset.preview;renderStreams();});
    $('stream-list').querySelectorAll('[data-stream]').forEach(el=>el.onchange=()=>{const [id,key]=el.dataset.stream.split(':');state.video.streams.find(s=>s.id===id)[key]=el.value;drawSlate();status('Stream assignment updated.');});
    $('stream-list').querySelectorAll('[data-receive]').forEach(el=>el.onchange=()=>{const [id,receiver]=el.dataset.receive.split(':');const s=state.video.streams.find(s=>s.id===id);s.receivers=el.checked?[...new Set([...s.receivers,receiver])]:s.receivers.filter(x=>x!==receiver);drawSlate();status('Receiver assignments updated.');});
    drawSlate();
  }
  function drawSlate(){
    const canvas=$('slate'),ctx=canvas.getContext('2d'),s=state.video.streams.find(s=>s.id===selectedStream);
    const w=Math.max(160,Math.min(3840,state.video.width)),h=Math.max(90,Math.min(2160,state.video.height));canvas.width=w;canvas.height=h;
    const scale=w/1280;ctx.save();ctx.scale(scale,h/720);ctx.fillStyle='#10252b';ctx.fillRect(0,0,1280,720);
    ctx.strokeStyle='#294148';ctx.lineWidth=1;for(let x=0;x<1280;x+=80){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,720);ctx.stroke();}for(let y=0;y<720;y+=80){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(1280,y);ctx.stroke();}
    ctx.fillStyle='#9adbc4';ctx.fillRect(64,62,8,42);ctx.font='500 40px sans-serif';ctx.fillText('MISSION SIMULATION',91,98);
    ctx.fillStyle='#eef5f1';ctx.font='600 96px sans-serif';ctx.fillText(s?(s.kind==='digital'?'DIGITAL ':'ANALOG ')+s.number:'NO STREAM',64,275,1152);
    ctx.fillStyle='#a8c2bd';ctx.font='40px sans-serif';ctx.fillText(s?s.vehicle:'Add a video stream to preview its slate.',68,335,1140);
    ctx.fillStyle='#d6e7e0';ctx.font='50px sans-serif';ctx.fillText(state.title,64,450,1152);
    ctx.fillStyle='#9eb7b2';ctx.font='46px monospace';ctx.fillText('#'+state.runId,64,512,1152);
    ctx.font='40px monospace';ctx.fillText(state.rocket.split(/[\\/]/).pop(),64,577,1152);
    ctx.font='24px monospace';ctx.fillText(s?s.id+'  |  STATIC IDENTIFICATION FRAME':'',64,660,1152);ctx.restore();
    $('slate-name').textContent=s?s.id+'.png · '+w+' × '+h:'No image';$('download-slate').disabled=!s;
  }
  function renderConnections(){
    $('receiver-site-choices').innerHTML=['gs1','gs2','gs3','gs4'].map((id,i)=>`<label><input type="checkbox" data-receiver-site="${id}" ${state.receiverSites.includes(id)||i<state.pointerCount?'checked':''} ${i<state.pointerCount?'disabled':''}>Away ${i+1}${i<state.pointerCount?' · pointer':''}</label>`).join('');
    $('receiver-site-choices').querySelectorAll('input').forEach(el=>el.onchange=()=>{const id=el.dataset.receiverSite;state.receiverSites=el.checked?[...new Set([...state.receiverSites,id])]:state.receiverSites.filter(x=>x!==id);renderSites();renderStreams();status('Receiver-only sites updated.');});
    $('connection-rows').innerHTML=stations().map((s,i)=>{
      const c=state.connections.stations[s.id]??={telemetry:s.id==='gs1',video:false,port:8766+i*2};
      return `<div class="connection-row"><div class="row-title">${s.id==='launch'?'Base station':siteName(s)}<small>${s.id} · simulation source</small></div><div class="checks"><label><input type="checkbox" data-station="${s.id}:telemetry" ${c.telemetry?'checked':''}> Telemetry</label><label><input type="checkbox" data-station="${s.id}:video" ${c.video?'checked':''}> Video</label></div><label>First service port<input type="number" min="1024" max="65534" step="1" data-station="${s.id}:port" value="${c.port}"></label></div>`;
    }).join('');
    $('connection-rows').querySelectorAll('input').forEach(el=>el.onchange=()=>{if(!el.checkValidity()){el.reportValidity();return;}const [id,key]=el.dataset.station.split(':');state.connections.stations[id][key]=el.type==='checkbox'?el.checked:Number(el.value);status('Station connection selection updated.');});
  }
  function review(){
    const warnings=[];
    for(const s of activeSites())if(!pointValid(s))warnings.push(siteName(s)+' needs coordinates.');
    for(const s of activeSites())if(!Number.isFinite(s.alt))warnings.push(siteName(s)+' needs an elevation; URRG presets contain horizontal positions only.');
    if(!state.title.trim())warnings.push('Enter a mission title.');
    if(!state.rocket.toLowerCase().endsWith('.ork'))warnings.push('Select a rocket .ork file.');
    if(state.telemetry.mode==='custom'&&!state.telemetry.fcPath)warnings.push('Select a flight computer definition.');
    if(state.telemetry.mode==='custom'&&!state.telemetry.definition&&state.telemetry.fallback==='block')warnings.push('The selected policy requires a telemetry format definition.');
    if(state.telemetry.mode==='custom'&&state.telemetry.definition)warnings.push('Custom telemetry needs a matching UI decoder; capability negotiation is not implemented in this preview.');
    if(state.flight.recovery==='Flight computer mappings'&&state.telemetry.mode==='standard')warnings.push('FC recovery authority requires an FC; standard telemetry is an observation generator.');
    const missing=[...new Set(state.video.streams.flatMap(s=>s.receivers).filter(id=>!stations().some(x=>x.id===id)))];
    if(missing.length)warnings.push('Video receivers need station locations: '+missing.join(', ')+'.');
    if(state.video.preset==='Custom')warnings.push('Custom video counts require the proposed Telemetry UI stream registry.');
    if(state.video.streams.some(s=>s.kind==='analog'&&s.receivers.includes('launch')))warnings.push('Base/Video currently accepts local Digital only; Base analog reception requires the proposed UI extension.');
    if(state.video.preset==='Iris'&&state.video.streams.some(s=>s.kind==='analog'&&((s.vehicle==='Sustainer'&&s.receivers.includes('gs4'))||(s.vehicle==='Booster'&&s.receivers.some(r=>r!=='gs4')))))warnings.push('Custom Iris analog receiver assignments differ from the current GS1–3 Sustainer / GS4 Booster mapping.');
    if(state.run.mode==='Interactive rehearsal'&&state.run.pace==='As fast as possible')warnings.push('Interactive UI connections require pacing or a negotiated lockstep endpoint.');
    if(state.run.mode==='Interactive rehearsal'&&state.run.type==='Ensemble')warnings.push('Attach the interactive UI to one selected ensemble member; run the other members headlessly.');
    const ports=[];for(const s of stations()){const c=state.connections.stations[s.id];if(c?.telemetry)ports.push(c.port);if(c?.video)ports.push(c.port+1);}
    if(new Set(ports).size!==ports.length)warnings.push('Enabled station service ports overlap.');
    warnings.push('Preview cannot verify file contents, geoid conversion, FC assignments or external endpoint compatibility.');
    const rows=[['Rocket',state.rocket],['Sites','Launch + Base + '+state.pointerCount+' pointers'],['Telemetry',state.telemetry.mode==='standard'?'Legacy Zephyrus · synthetic observation source':state.telemetry.definition||'Legacy Zephyrus fallback'],['Video',state.video.digital+' digital · '+state.video.analog+' analog'],['Execution',state.run.mode+' · '+(state.run.type==='Ensemble'?state.run.count+' flights':state.run.type)],['Bundle',state.run.filename]];
    $('review').innerHTML=rows.map(([k,v])=>`<div class="review-item"><span>${k}</span><span>${esc(v)}</span></div>`).join('')+warnings.map(x=>`<div class="review-warning">${esc(x)}</div>`).join('');
    return warnings;
  }
  function updateDerived(){
    const usesUrrg=activeSites().every(s=>s.coordinateInput?.source==='telemetry-ui-urrg');
    $('preset-name').textContent=usesUrrg?'URRG preset':'Custom locations';
    if(page==='site')$('page-description').textContent=usesUrrg?descriptions.site:'Custom mission locations · saved coordinates remain authoritative.';
    $('sidebar-summary').textContent=state.pointerCount+' pointers · '+state.video.streams.length+' video streams · '+(state.run.type==='Ensemble'?state.run.count+' flights':'Single flight');
    $('custom-fc').hidden=state.telemetry.mode!=='custom';$('ensemble-settings').hidden=state.run.type!=='Ensemble';$('wind-levels').hidden=state.flight.windModel!=='Multi-level';
    document.querySelector('[data-bind="telemetry.rate"]').disabled=state.telemetry.mode==='custom';
    const into=document.querySelector('[data-bind="flight.bearing"]');if(into)into.disabled=state.flight.intoWind;
    for(const key of ['temperature','pressure']){const el=document.querySelector('[data-bind="flight.'+key+'"]');if(el)el.disabled=state.flight.atmosphere==='ISA';}
    const standard=state.telemetry.mode==='standard'||!state.telemetry.definition;
    $('packet-summary').textContent=standard?'Legacy Zephyrus: 128-byte payload → 144-byte receiver frame at '+state.telemetry.rate+' Hz. Broadcast to every configured station; receiver loss and delay apply independently. '+(state.telemetry.mode==='standard'?'Synthetic sensor observations; no custom FC or onboard control is implied.':'Fallback encoding must preserve the selected FC’s available signals.'):'Declared format: '+state.telemetry.definition+'. Validate the encoder and receiving UI decoder before running. No silent conversion to legacy fields.';
    drawSlate();if(page==='run')review();
  }
  function download(name,blob){const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  function save(){download((state.title.trim().toLowerCase().replace(/[^a-z0-9]+/g,'-')||'mission')+'.rktmission-config.json',new Blob([JSON.stringify(state,null,2)+'\n'],{type:'application/json'}));status('Configuration exported · load this file to restore the preview.');}
  function validateImport(v){
    if(v?.format!==initial.format||v.version!==1)throw Error('Choose a version 1 mission configuration from this preview.');
    if(!Number.isInteger(v.pointerCount)||v.pointerCount<0||v.pointerCount>4)throw Error('Pointer count must be 0–4.');
    if(!Array.isArray(v.sites)||v.sites.length!==6||!v.sites.every((s,i)=>s.id===initial.sites[i].id&&(s.lat===null&&s.lon===null||pointValid(s))&&(s.alt===null||Number.isFinite(s.alt))))throw Error('Invalid station coordinates or IDs.');
    for(const k of ['title','runId','rocket'])if(typeof v[k]!=='string'||v[k].length>500)throw Error('Invalid '+k+'.');
    for(const k of ['flight','telemetry','video','connections','run'])if(!v[k]||typeof v[k]!=='object'||Array.isArray(v[k]))throw Error('Missing '+k+' settings.');
    for(const kind of ['digital','analog'])if(!Number.isInteger(v.video[kind])||v.video[kind]<0||v.video[kind]>32)throw Error('Video counts must be integers from 0–32 in this preview.');
    if(!Array.isArray(v.video.streams)||v.video.streams.length>64)throw Error('Invalid stream list.');
    const merge=(base,value)=>{if(Array.isArray(base))return structuredClone(value);if(base&&typeof base==='object'){const out={};for(const k of Object.keys(base))out[k]=Object.hasOwn(value,k)?merge(base[k],value[k]):structuredClone(base[k]);return out;}if(typeof value!==typeof base)throw Error('Invalid configuration field type.');return value;};
    const safe=merge(initial,v);
    safe.receiverSites=Array.isArray(v.receiverSites)?[...new Set(v.receiverSites.filter(x=>['gs1','gs2','gs3','gs4'].includes(x)))]:[];
    safe.sites=v.sites.map((s,i)=>({...initial.sites[i],lat:s.lat,lon:s.lon,alt:s.alt,datum:s.datum==='ellipsoid'?'ellipsoid':'MSL',coordinateInput:s.coordinateInput?.format==='mgrs'&&typeof s.coordinateInput.value==='string'&&/^[0-9A-Z]{5,15}$/.test(s.coordinateInput.value)?{format:'mgrs',value:s.coordinateInput.value,reference:s.coordinateInput.reference==='grid-point'?'grid-point':'cell-center',precisionM:Number.isFinite(s.coordinateInput.precisionM)?s.coordinateInput.precisionM:1,source:s.coordinateInput.source==='telemetry-ui-urrg'?'telemetry-ui-urrg':'manual'}:{format:'latlon',source:s.coordinateInput?.source==='map'?'map':'manual'},...Object.fromEntries(['height','azRate','elRate','az','el'].filter(k=>Number.isFinite(s[k])).map(k=>[k,s[k]]))}));
    safe.telemetry.overrides={};safe.connections.stations={};
    for(const s of safe.sites.slice(1)){
      const o=v.telemetry.overrides?.[s.id];if(o&&Number.isFinite(o.loss)&&o.loss>=0&&o.loss<=100&&Number.isFinite(o.delay)&&o.delay>=0)safe.telemetry.overrides[s.id]={enabled:Boolean(o.enabled),loss:o.loss,delay:o.delay};
      const c=v.connections.stations?.[s.id];if(c&&Number.isInteger(c.port)&&c.port>=1024&&c.port<=65534)safe.connections.stations[s.id]={telemetry:Boolean(c.telemetry),video:Boolean(c.video),port:c.port};
    }
    safe.video.streams=v.video.streams.filter(s=>['digital','analog'].includes(s.kind)&&s.id===s.kind+'-'+s.number&&Number.isInteger(s.number)&&s.number>0&&s.number<=32&&['Balius','Sustainer','Booster','Unassigned'].includes(s.vehicle)&&Array.isArray(s.receivers)&&s.receivers.every(r=>safe.sites.slice(1).some(x=>x.id===r))).map(s=>({id:s.id,kind:s.kind,number:s.number,vehicle:s.vehicle,receivers:[...new Set(s.receivers)]}));
    return safe;
  }
  document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>showPage(b.dataset.page));
  $('next-page').onclick=()=>showPage(pages[(pages.indexOf(page)+1)%pages.length]);
  $('pointer-count').onchange=()=>{state.pointerCount=Number($('pointer-count').value);placing=false;renderSites();renderStreams();status('Pointer count updated · removed pointer settings are retained in this draft.');};
  $('mode-latlon').onclick=()=>coordinateMode('latlon');$('mode-mgrs').onclick=()=>coordinateMode('mgrs');$('apply-coordinates').onclick=applyCoordinates;
  $('place-site').onclick=()=>{placing=!placing;renderSites();$('map-help').textContent=placing?'Click the map to place '+site().name+'.':'Drag a marker, or select a site and choose Place on map.';};
  $('fit-sites').onclick=()=>{if(map){const points=activeSites().filter(pointValid).map(s=>[s.lat,s.lon]);if(points.length)map.fitBounds(points,{padding:[45,45],maxZoom:15});}};
  $('map-layer').onchange=setLayer;
  $('video-preset').onchange=()=>{state.video.preset=$('video-preset').value;if(state.video.preset!=='Custom'){state.video.digital=1;state.video.analog=state.video.preset==='Iris'?2:1;}createStreams(true);renderStreams();updateDerived();};
  for(const kind of ['digital','analog'])$(kind+'-count').onchange=()=>{const el=$(kind+'-count');if(!el.checkValidity()||el.value===''){el.reportValidity();return;}state.video[kind]=Number(el.value);state.video.preset='Custom';createStreams();renderStreams();updateDerived();};
  $('download-slate').onclick=()=>{$('slate').toBlob(blob=>{if(blob)download(state.runId+'-'+selectedStream+'.png',blob);},'image/png');};
  $('save-config').onclick=save;$('export-review').onclick=save;$('load-config').onclick=()=>$('config-file').click();
  $('config-file').onchange=async()=>{try{const f=$('config-file').files[0];if(!f)return;if(f.size>1024*1024)throw Error('Configuration exceeds the 1 MB preview limit.');const candidate=validateImport(JSON.parse(await f.text()));state=candidate;createStreams();bind();renderSites();renderStreams();showPage(page);$('fit-sites').click();status('Loaded '+f.name+'.');}catch(e){status('Configuration not loaded: '+e.message);}$('config-file').value='';};
  $('choose-rocket').onclick=()=>$('rocket-file').click();$('rocket-file').onchange=()=>{const f=$('rocket-file').files[0];if(f){state.rocket=f.name;bind();updateDerived();status('Rocket filename selected · this preview does not parse the .ork contents.');}};
  $('choose-fc').onclick=()=>$('fc-file').click();$('fc-file').onchange=()=>{const f=$('fc-file').files[0];if(f){state.telemetry.fcPath=f.name;state.telemetry.definition='';bind();updateDerived();status('FC filename selected · format/decoder validation belongs to the integration.');}};
  $('review-button').onclick=()=>{const notes=review();status('Configuration reviewed · '+notes.length+' notes to resolve or verify. No simulation started.');};
  buildForms();createStreams();bind();initMap();coordinateMode(coordMode);renderSites();renderStreams();showPage('site');
  // Exposed only for reproducible design-preview checks, never for application integration.
  window.missionPreview={snapshot:()=>structuredClone(state),validateImport,showPage};
})();
