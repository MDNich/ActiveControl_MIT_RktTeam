const {chromium}=require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs=require('fs');const assert=require('assert/strict');const path=require('path');
const root=path.resolve(process.argv[2]);
(async()=>{
 const browser=await chromium.launch({headless:true,...(process.env.BROWSER_EXECUTABLE?{executablePath:process.env.BROWSER_EXECUTABLE}:{})});
 const page=await browser.newPage({viewport:{width:1440,height:1100},colorScheme:'light'});const errors=[];
 page.on('pageerror',e=>errors.push(e.message));
 await page.route('https://**/*',r=>r.abort()); // Numerical editor must work with all imagery unavailable.
 await page.goto(fs.readFileSync(path.join(root,'workspace-url'),'utf8'));
 await page.locator('#status').filter({hasText:'Mission workspace connected.'}).waitFor();
 assert.equal(await page.locator('#site-list button').count(),6);
 await page.selectOption('#map-layer','none');
 await page.locator('#apply-mgrs').click();assert.match(await page.locator('#status').innerText(),/Original saved point retained/);
 await page.screenshot({path:path.join(root,'site.png'),fullPage:true});
 for(const site of ['launch-site','launch','gs1','gs2','gs3','gs4']){
  await page.locator(`[data-site="${site}"]`).click();await page.locator('[data-field$=".heightM"]').fill('250');await page.locator('[data-field$=".heightM"]').press('Tab');
 }
 await page.locator('[data-page="flight"]').click();
 assert.equal(await page.locator('[data-field^="flight.editor.options."]').count()>20,true);
 await page.locator('[data-field="flight.editor.options.guideLengthM"]').fill('7.25');await page.locator('[data-field="flight.editor.options.guideLengthM"]').press('Tab');
 await page.locator('[data-page="telemetry"]').click();await page.locator('[data-field="telemetry.links.0.packetLossFraction"]').fill('12.5');await page.locator('[data-field="telemetry.links.0.packetLossFraction"]').press('Tab');
 await page.locator('[data-page="video"]').click();await page.selectOption('[data-field="video.preset"]','Iris');
 assert.equal(await page.locator('.stream-card').count(),3);
 assert.equal(await page.locator('[data-field="video.streams.0.vehicleId"]').inputValue(),'sustainer');
 assert.equal(await page.locator('[data-receiver="0:gs4"]').isChecked(),true);
 assert.equal(await page.locator('[data-receiver="1:gs4"]').isChecked(),false);
 assert.equal(await page.locator('[data-receiver="2:gs4"]').isChecked(),true);
 await page.locator('#digital-count').fill('2');await page.locator('#digital-count').press('Tab');assert.equal(await page.locator('.stream-card').count(),4);
 await page.screenshot({path:path.join(root,'video.png'),fullPage:true});
 await page.selectOption('[data-field="video.preset"]','Balius');
 await page.locator('[data-page="run"]').click();await page.locator('#validate').click();
 await page.locator('#findings').filter({hasText:'Configuration inputs are valid.'}).waitFor();
 assert.equal(await page.locator('#start').isDisabled(),true);
 fs.writeFileSync(path.join(root,'chooser-next-path'),path.join(root,'saved.rktmission-config.json'));
 await page.locator('#save').click();await page.locator('#status').filter({hasText:'Saved saved.rktmission-config.json'}).waitFor();
 const saved=JSON.parse(fs.readFileSync(path.join(root,'saved.rktmission-config.json'),'utf8'));
 assert.equal(saved.flight.editor.options.guideLengthM,7.25);assert.equal(saved.telemetry.links[0].packetLossFraction,.125);assert.equal(saved.sites[0].latitudeDeg,42.70416776971639);
 const identity=saved.identity.missionId;await page.locator('#load').click();await page.locator('#status').filter({hasText:'Configuration loaded.'}).waitFor();
 const current=await page.evaluate(async()=>{const {state}=await import('./api.js');return state.mission;});assert.equal(current.identity.missionId,identity);assert.equal(current.flight.editor.options.guideLengthM,7.25);
 assert.deepEqual(errors,[]);console.log('Browser acceptance passed: offline map, exact coordinate reapply, six heights, flight/link edits, Iris routes, Digital 2, validation, save/load identity.');
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1);});
