import {state,$,esc,changed,action,request,status} from './api.js';
import {field,attachFields} from './forms.js';
let map,layer,markers=[],selected='launch-site',placing=false;
const names={'launch-site':'Launch site',launch:'Base station',gs1:'Away 1',gs2:'Away 2',gs3:'Away 3',gs4:'Away 4'};
export function renderSites() {
  if(!map){
    map=L.map('map',{zoomControl:true}).setView([42.70416776971639,-77.19029501648315],15);
    L.control.scale({imperial:false}).addTo(map);map.on('click',event=>{if(placing){move(selected,event.latlng.lat,event.latlng.lng);placing=false;renderSites();}});
    $('fit-sites').onclick=fitSites;$('map-layer').onchange=changeLayer;changeLayer();
    $('pointer-count').onchange=()=>{
      const count=Number($('pointer-count').value);
      for(let i=0;i<4;i++){
        let pointer=state.mission.pointers.find(p=>p.siteId==='gs'+(i+1));
        if(!pointer){pointer={id:'pointer'+(i+1),siteId:'gs'+(i+1),enabled:false,targetVehicleId:state.mission.vehicleBindings[0].id,trackingPolicy:'geodetic.v1',initialAzimuthDeg:0,initialElevationDeg:0,azimuthRateDegS:90,elevationRateDegS:60,mountHeightM:2};state.mission.pointers.push(pointer);}
        pointer.enabled=i<count;
        if(pointer.enabled)state.mission.sites.find(s=>s.id===pointer.siteId).enabled=true;
      }
      changed();renderSites();
    };
    setTimeout(fitSites,50);
  }
  $('pointer-count').value=state.mission.pointers.filter(p=>p.enabled).length;
  if(!state.mission.sites.some(s=>s.id===selected))selected='launch-site';
  $('site-list').innerHTML=state.mission.sites.map(s=>`<button class="${s.id===selected?'selected':''}" data-site="${s.id}"><strong>${names[s.id]||esc(s.name)}</strong><small>${s.enabled?(s.heightM==null?'Height needed':'Located'):'Disabled'}</small></button>`).join('');
  $('site-list').querySelectorAll('button').forEach(b=>b.onclick=()=>{selected=b.dataset.site;renderSites();});
  markers.forEach(m=>m.remove());markers=[];
  for(const site of state.mission.sites){
    const symbol=site.id==='launch-site'?'L':site.id==='launch'?'B':site.id.slice(2);
    const css=site.id==='launch-site'?'launch':site.id==='launch'?'base':'';
    const marker=L.marker([site.latitudeDeg,site.longitudeDeg],{draggable:!(state.mission.recordedFlight&&site.id==='launch-site'),icon:L.divIcon({className:'',html:`<div class="map-pin ${css} ${site.id===selected?'selected':''} ${site.enabled?'':'disabled'}">${symbol}</div>`,iconSize:[30,30],iconAnchor:[15,15]})}).addTo(map);
    const tooltip=document.createElement('span');tooltip.textContent=names[site.id]||site.name;marker.bindTooltip(tooltip);
    marker.setZIndexOffset(site.id===selected?1000:0);
    marker.on('click',()=>{selected=site.id;renderSites();});marker.on('dragend',()=>{const p=marker.getLatLng();move(site.id,p.lat,p.lng);selected=site.id;renderSites();});markers.push(marker);
  }
  renderInspector();setTimeout(()=>map.invalidateSize(),0);
}
function renderInspector(){
  const i=state.mission.sites.findIndex(s=>s.id===selected),site=state.mission.sites[i],p=`sites.${i}.`;
  const pi=state.mission.pointers.findIndex(p=>p.siteId===site.id),pointer=state.mission.pointers[pi];
  let html=`<h2>${names[site.id]||esc(site.name)}</h2>`;
  if(site.id.startsWith('gs'))html+=field(p+'enabled','Enable receiver at this site');
  html+=`<div class="two-col">${field(p+'latitudeDeg','Latitude · °',{type:'number',min:-90,max:90})}${field(p+'longitudeDeg','Longitude · °',{type:'number',min:-180,max:180})}</div><label>MGRS reference<input id="mgrs-input" value="${site.coordinateInput?.format==='mgrs'?esc(site.coordinateInput.value):''}" placeholder="18TUN2061530290" autocomplete="off"></label><button id="apply-mgrs" class="wide">Apply MGRS</button><p class="hint">New MGRS entries use the grid point. Reapplying a saved reference retains its original point.</p><div class="two-col">${field(p+'heightM','Ground height · m',{type:'number',min:-1000,max:100000})}${field(p+'heightDatum','Height reference',{choices:[['MSL','Mean sea level'],['ellipsoidal','WGS84 ellipsoid']]})}</div><button id="place-site" class="wide">${placing?'Click the map…':'Place on map'}</button><p class="hint">Enter a measured or chosen ground height. Antenna height is added separately.</p>`;
  if(pointer){const pp=`pointers.${pi}.`;html+='<details open><summary>Pointer settings</summary>'+field(pp+'enabled','Enable antenna pointer')+field(pp+'targetVehicleId','Tracking target',{choices:state.mission.vehicleBindings.map(v=>[v.id,v.id])})+field(pp+'trackingPolicy','Tracking policy',{choices:[['geodetic.v1','Geodetic · absolute coordinates'],['legacy-ui.v1','Legacy UI profile']]})+field(pp+'mountHeightM','Antenna above ground · m',{type:'number',min:0})+`<div class="two-col">${field(pp+'initialAzimuthDeg','Initial azimuth · °',{type:'number'})}${field(pp+'initialElevationDeg','Initial elevation · °',{type:'number',min:0,max:90})}${field(pp+'azimuthRateDegS','Azimuth rate · °/s',{type:'number',min:.001})}${field(pp+'elevationRateDegS','Elevation rate · °/s',{type:'number',min:.001})}</div><p class="hint">Default slew rates are illustrative. They are not a measured mount profile.</p></details>`;}
  $('site-editor').innerHTML=html;attachFields($('site-editor'));
  if(state.mission.recordedFlight&&site.id==='launch-site'){$('site-editor').insertAdjacentHTML('afterbegin','<p class=information>Launch location is fixed by the recorded trajectory.</p>');$('site-editor').querySelectorAll('input,select,button').forEach(el=>el.disabled=true);placing=false;}
  $('apply-mgrs').onclick=()=>action(async()=>{
    const input=$('mgrs-input').value,normalize=s=>s.replace(/\s+/g,'').toUpperCase();
    if(site.coordinateInput?.format==='mgrs'&&normalize(site.coordinateInput.value)===normalize(input)){status('Original saved point retained.');return;}
    Object.assign(site,await request('coordinates',{mgrs:input,reference:'grid-point'}));changed();renderSites();status('MGRS position applied.');
  });
  $('place-site').onclick=()=>{placing=!placing;renderInspector();status(placing?'Click the map to place '+names[site.id]+'.':'Map placement cancelled.');};
}
function move(id,lat,lon){if(state.mission.recordedFlight&&id==='launch-site')return;const site=state.mission.sites.find(s=>s.id===id);site.latitudeDeg=Math.max(-90,Math.min(90,lat));site.longitudeDeg=((lon+540)%360)-180;site.coordinateInput={format:'latlon',value:`${site.latitudeDeg},${site.longitudeDeg}`,reference:'point',source:'map'};changed();}
function fitSites(){if(!map)return;const sites=state.mission.sites.filter(s=>s.enabled);if(sites.length)map.fitBounds(sites.map(s=>[s.latitudeDeg,s.longitudeDeg]),{padding:[45,45],maxZoom:17});}
function changeLayer(){
  if(layer)layer.remove();const type=$('map-layer').value;
  if(type==='none'){$('map-status').textContent='Offline coordinate map';return;}
  const providers={hybrid:['https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryTopo/MapServer/tile/{z}/{y}/{x}','USGS · The National Map'],satellite:['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}','Tiles © Esri · Maxar · Earthstar Geographics · GIS User Community'],street:['https://tile.openstreetmap.org/{z}/{x}/{y}.png','© <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noreferrer">OpenStreetMap</a> contributors']};
  const [url,attribution]=providers[type];$('map-status').textContent='Loading imagery…';
  layer=L.tileLayer(url,{maxZoom:19,attribution,keepBuffer:1});let failed=false;layer.on('tileerror',()=>{failed=true;$('map-status').textContent='Imagery unavailable · coordinates remain usable';});layer.on('load',()=>{if(!failed)$('map-status').textContent='Imagery loaded';});layer.addTo(map);
}
document.addEventListener('field-changed',event=>{
  const path=event.detail;if(!path.startsWith('sites.')&&!path.startsWith('pointers.'))return;
  if(/sites\.\d+\.(latitudeDeg|longitudeDeg)$/.test(path)){const s=state.mission.sites[Number(path.split('.')[1])];s.coordinateInput={format:'latlon',value:`${s.latitudeDeg},${s.longitudeDeg}`,reference:'point',source:'manual'};}
  if(path.endsWith('.enabled')){
    for(const pointer of state.mission.pointers){const site=state.mission.sites.find(s=>s.id===pointer.siteId);if(path.startsWith('sites.')&&!site.enabled)pointer.enabled=false;else if(pointer.enabled)site.enabled=true;}
  }
  renderSites();
});
