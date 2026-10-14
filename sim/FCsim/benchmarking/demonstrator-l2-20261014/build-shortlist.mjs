import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { Workbook } from '@oai/artifact-tool';

const source = '/Users/mdn/Developer/MIT_Rkt_Team/2026-7/MIT-RT-26_27-Work/simulation_work/generic/openrocket/OpenRocket2026/2627_official/20267.demonstrator.iteration4.ork';
const sourceHash = '80dccac0df5df058cead0c21932ff1e76efb961168ee17e49e2e2f0a8b804788';
assert.equal(createHash('sha256').update(await fs.readFile(source)).digest('hex'), sourceHash);
const report = JSON.parse(await fs.readFile('conditional/motor-batch-results.json', 'utf8'));
const choices = ['J1265T','HP-J425R','HP-J270W','J435WS','J450DM','J350W','J500G','J1299N','J1799N','J540R'];
const selected = choices.map(designation => {
  const row = report.rows.find(r => r.motor.designation === designation);
  assert.ok(row, designation);
  assert.equal(row.selected.status, 'COMPLETE');
  assert.ok(row.selected.metrics.APOGEE < 914.4);
  assert.equal(row.exclusion, '');
  assert.ok(row.ignoredIntersections.every(x => /^MOTOR_INTERSECTS:(RCR|RRC) \[/.test(x)));
  assert.ok(row.motor.length <= .397);
  assert.ok(['SINGLE','RELOAD'].includes(row.motor.type));
  return row;
});
const values = [['Designation','Diameter (mm)','SingleUse/reload type'], ...selected.map(r => [
  `AeroTech ${r.motor.designation === 'J1265T' ? 'J1265ST' : r.motor.designation.replace(/^HP-/, '')}`,
  Math.round(r.motor.diameter * 1000),
  r.motor.type === 'SINGLE' ? 'Single-use' : 'Reload'
])];
const workbook = Workbook.create();
const sheet = workbook.worksheets.add('L2 motors');
sheet.showGridLines = false;
sheet.getRange('A1:C11').values = values;
sheet.getRange('A1:C11').format.font = {name:'Arial',size:11,color:'#17212B'};
sheet.getRange('A1:C11').format.rowHeight = 24;
sheet.getRange('A1:C11').format.verticalAlignment = 'center';
sheet.getRange('A1:C1').format.fill = '#263C50';
sheet.getRange('A1:C1').format.font = {name:'Arial',size:11,bold:true,color:'#FFFFFF'};
sheet.getRange('A1:C1').format.horizontalAlignment = 'center';
sheet.getRange('A1:A11').format.columnWidth = 25;
sheet.getRange('B1:B11').format.columnWidth = 18;
sheet.getRange('C1:C11').format.columnWidth = 27;
sheet.getRange('B2:B11').setNumberFormat('0');
sheet.getRange('B2:B11').format.horizontalAlignment = 'right';
workbook.recalculate();
console.log((await workbook.inspect({kind:'table',range:"'L2 motors'!A1:C11",include:'values,formulas',tableMaxRows:11,tableMaxCols:3,maxChars:2500})).ndjson);
console.log((await workbook.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:20}})).ndjson);
const preview = await workbook.render({sheetName:'L2 motors',range:'A1:C11',scale:2,format:'png'});
await fs.writeFile('shortlist-preview.png',new Uint8Array(await preview.arrayBuffer()));
// The requested deliverable is CSV: serialize the verified, typed worksheet values.
const actual = sheet.getRange('A1:C11').values;
assert.deepEqual(actual, values);
const escape = value => /[",\r\n]/.test(String(value)) ? '"' + String(value).replaceAll('"','""') + '"' : String(value);
const csv = actual.map(row => row.map(escape).join(',')).join('\r\n') + '\r\n';
await fs.writeFile('demonstrator-l2-motor-shortlist.csv', csv);
const reopened = await Workbook.fromCSV(await fs.readFile('demonstrator-l2-motor-shortlist.csv','utf8'),{sheetName:'Check'});
assert.deepEqual(reopened.worksheets.getItem('Check').getRange('A1:C11').values,values.map(r => r.map(String)));
await fs.writeFile('shortlist-verification.json',JSON.stringify({
  source,sourceHash,baseline:report.input.baseline,applicationVersion:report.input.version,
  assumptions:[
    'Current saved geometry, mass, 40-degree roll tabs, and launch conditions retained.',
    'Apogee plus 1 second recovery retained; unused motor ejection events disabled on analysis copy.',
    'User confirmed on 2026-10-14 that actual RCR/RRC hardware may be assumed to clear 54 mm motors; no other component intersections waived.',
    'Nominal simulated apogee under 3000 ft AGL; not an uncertainty upper bound.'
  ],
  sources:[
    'https://aerotech-rocketry.com/collections/all/hazmat?page=3',
    'https://tripoli.org/content.aspx?club_id=795696&item_id=83506&page_id=5'
  ],
  rows:selected.map(r=>({designation:r.motor.designation,diameterMm:r.motor.diameter*1000,
    lengthMm:r.motor.length*1000,type:r.motor.type,curveDigest:r.motor.digest,
    apogeeFt:r.selected.metrics.APOGEE/.3048,railExitMs:r.selected.metrics.RAIL_SPEED,
    waivedPlaceholderIntersections:r.ignoredIntersections}))
},null,2)+'\n');
console.log('Saved and verified 10 motors, 3 columns; source rocket unchanged.');
