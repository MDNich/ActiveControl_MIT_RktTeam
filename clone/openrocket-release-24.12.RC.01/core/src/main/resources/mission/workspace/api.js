const parameters = new URLSearchParams(location.hash.slice(1));
if (parameters.has('token')) sessionStorage.setItem('mission-token', parameters.get('token'));
if (parameters.has('draft')) sessionStorage.setItem('mission-draft', parameters.get('draft'));
history.replaceState(null, '', location.pathname);
export const state = {mission:null, capabilities:null, preferences:{}, dirty:false, page:'site', resolved:null, job:null};
export const $ = id => document.getElementById(id);
export const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export const get = path => path.split('.').reduce((o,k) => o?.[k.replaceAll('~1','.').replaceAll('~0','~')], state.mission);
export function status(text) { $('status').textContent = text; }
export function changed() { state.dirty = true; state.resolved = null; $('start').disabled = true; $('change-status').textContent = 'Changes are not yet applied.'; $('draft-state').textContent = 'Unsaved changes'; }
export function set(path, value) { const keys=path.split('.').map(k=>k.replaceAll('~1','.').replaceAll('~0','~')), last=keys.pop(); let o=state.mission; for(const key of keys) o=o[key]??=(/^\d+$/.test(key)?[]:{}); o[last]=value; changed(); }
export async function request(path, body, method=body===undefined?'GET':'POST') {
  const token=sessionStorage.getItem('mission-token');
  if(!token) throw Error('Open Mission Simulation from the OpenRocket File menu to connect this workspace.');
  const response=await fetch('/mission-api/v1/'+path,{method,headers:{'X-Mission-Token':token,...(body===undefined?{}:{'Content-Type':'application/json'})},body:body===undefined?undefined:JSON.stringify(body)});
  const data=await response.json(); if(!response.ok) throw Error(data.error||'The operation could not be completed.'); return data;
}
export function useMission(mission) { state.mission=mission; state.dirty=false; state.resolved=null; sessionStorage.setItem('mission-draft',mission.identity.missionId); document.dispatchEvent(new Event('mission-loaded')); }
export async function commit() {
  const invalid=document.querySelector('input:invalid,select:invalid,textarea:invalid');
  if(invalid){invalid.reportValidity();throw Error('Correct the invalid field before applying or running this configuration.');}
  if(!state.dirty) return;
  const mission=await request('drafts/'+state.mission.identity.missionId,{expectedRevision:state.mission.identity.revision,mission:state.mission},'PUT');
  useMission(mission); status('Changes applied to the mission draft.');
}
export async function action(operation) {
  $('error').hidden=true; $('workspace').inert=true;
  try { return await operation(); }
  catch(error) { $('error').hidden=false; $('error').textContent=error.message; status(error.message); return undefined; }
  finally { $('workspace').inert=false; }
}
export async function draftAction(name, extra={}) { await commit(); return request('drafts/'+state.mission.identity.missionId+'/'+name,{expectedRevision:state.mission.identity.revision,...extra}); }
export async function initialize() {
  state.capabilities=await request('capabilities'); state.preferences=await request('preferences');
  const id=sessionStorage.getItem('mission-draft'); useMission(id?await request('drafts/'+id):await request('drafts',{title:'URRG mission'}));
}
window.addEventListener('beforeunload',event=>{if(state.dirty){event.preventDefault();event.returnValue='';}});
