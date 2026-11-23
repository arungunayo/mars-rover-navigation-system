const fs=require('fs'),vm=require('vm'),assert=require('assert');
const file='C:/Users/aroha/.codex/visualizations/2026/11/22/01a203ad-33c4-7f00-b321-6811e11da839/rover-point-map.html';
const html=fs.readFileSync(file,'utf8');
const dataText=html.match(/<script type="application\/json" id="rover-point-map-data">([\s\S]*?)<\/script>/)[1];
const data=JSON.parse(dataText),code=html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];
function element(){return{style:{},handlers:{},value:'',textContent:'',setAttribute(k,v){this[k]=v},addEventListener(k,v){this.handlers[k]=v},focus(){},select(){},remove(){}}}
const context=new Proxy({getImageData(){return{data:[32,90,150,255]}}},{get(t,k){return k in t?t[k]:(()=>{})},set(t,k,v){t[k]=v;return true}});
const canvas=element();canvas.getContext=()=>context;canvas.getBoundingClientRect=()=>({left:0,top:0,width:640,height:640});
const nodes={};for(const key of ['x','y','command','status','detail','apply','copy','copy-status','coordinates','low','high']){assert(html.includes('data-'+key));nodes['[data-'+key+']']=element()}
const modes=['start','goal'].map(mode=>({...element(),dataset:{mode}}));
const root=element();root.querySelector=s=>{if(s==='canvas')return canvas;if(!nodes[s])throw Error('Missing selector '+s);return nodes[s]};root.querySelectorAll=()=>modes;root.append=()=>{};
let saved;
const sandbox={document:{getElementById(id){if(id==='rover-point-map')return root;if(id==='rover-point-map-data')return{textContent:dataText};throw Error('Missing id '+id)},createElement(tag){if(tag==='canvas')return{getContext:()=>context};return element()},documentElement:{}},getComputedStyle:()=>({color:'rgb(32,90,150)',getPropertyValue:()=> '14'}),window:{devicePixelRatio:1,addEventListener(){},openai:{setWidgetState(s){saved=s;return Promise.resolve()},sendFollowUpMessage(){return Promise.resolve()}}},navigator:{clipboard:{writeText(){return Promise.resolve()}}},ResizeObserver:class{observe(){}},MutationObserver:class{observe(){}},console};
vm.createContext(sandbox);vm.runInContext(code,sandbox);
assert(nodes['[data-status]'].textContent.includes('connected grid route'),'Selected real endpoints must be connected');
assert(!nodes['[data-copy]'].disabled,'Reachable current endpoints must enable apply/copy');
assert(nodes['[data-command]'].value.includes('-TerrainProfile original-reduced -SlopeReductionDegrees 3'),'Command must preserve restored profile');
const initial=nodes['[data-command]'].value;
function safe(x,y){const i=y*65+x,wx=-25+x*data.spacing,wy=-25+y*data.spacing;return Math.hypot((data.heights[i+1]-data.heights[i-1])/(2*data.spacing),(data.heights[i+65]-data.heights[i-65])/(2*data.spacing))<.4&&data.rocks.every(r=>Math.hypot(wx-r.x,wy-r.y)>r.radius*1.15+.65+data.spacing)}
let pair;
for(let y=4;y<61&&!pair;y++)for(let x=4;x<60;x++)if(safe(x,y)&&safe(x+1,y)&&Math.abs(data.heights[y*65+x+1]-data.heights[y*65+x])/data.spacing<.4){pair={x,y};break}
assert(pair,'Terrain must contain a valid adjacent-point fixture');
const sx=-25+pair.x*data.spacing,sy=-25+pair.y*data.spacing;
modes[0].handlers.click();
canvas.handlers.click({clientX:42+(sx+25)/50*578,clientY:12+(25-sy)/50*578});
modes[1].handlers.click();
const gx=sx+data.spacing,gy=sy;
canvas.handlers.click({clientX:42+(gx+25)/50*578,clientY:12+(25-gy)/50*578});
assert(nodes['[data-command]'].value!==initial,'Map click must update command');
assert(saved.modelContent.goal.y===gy,'Map click must save chosen coordinate');
assert(nodes['[data-copy]'].disabled===false,'Valid chosen endpoint should remain usable');
let cliff=-1;
for(let y=4;y<61&&cliff<0;y++)for(let x=4;x<61;x++){const i=y*65+x,gx=(data.heights[i+1]-data.heights[i-1])/(2*data.spacing),gy=(data.heights[i+65]-data.heights[i-65])/(2*data.spacing);if(Math.hypot(gx,gy)>.8){cliff=i;break}}
if(cliff<0){const r=data.rocks[0];cliff=Math.round((r.y+25)/data.spacing)*65+Math.round((r.x+25)/data.spacing)}
assert(cliff>=0,'Fixture has a restricted endpoint');
const x=-25+(cliff%65)*data.spacing,y=-25+Math.floor(cliff/65)*data.spacing;
canvas.handlers.click({clientX:42+(x+25)/50*578,clientY:12+(25-y)/50*578});
assert(nodes['[data-copy]'].disabled,'Unsafe selection must not generate an enabled command action');
assert(nodes['[data-status]'].textContent.includes('No connected route'));
console.log('[VERIFY] actual-map connectivity, coordinate click, command update, saved selection, and restricted-endpoint checks passed; no browser or screen access used');
