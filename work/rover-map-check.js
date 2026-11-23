(()=>{
    const root=document.getElementById('rover-point-map'),data=JSON.parse(document.getElementById('rover-point-map-data').textContent);
    const canvas=root.querySelector('canvas'),ctx=canvas.getContext('2d'),qx=root.querySelector('[data-x]'),qy=root.querySelector('[data-y]');
    const command=root.querySelector('[data-command]'),status=root.querySelector('[data-status]'),detail=root.querySelector('[data-detail]');
    const apply=root.querySelector('[data-apply]'),copy=root.querySelector('[data-copy]'),copyStatus=root.querySelector('[data-copy-status]');
    const n=data.width,s=data.spacing,ox=data.originX,oy=data.originY,z=data.heights.map(h=>h+data.originZ),rocks=data.rocks;
    const maxSlope=data.plannerMaxGrade??.3;
    let start={...data.start},goal={...data.goal},mode='start',hover=null,frame=null,colors={},ramp=[],valid=false;
    const low=Math.min(...z),high=Math.max(...z),grade=new Float64Array(z.length),safe=new Uint8Array(z.length);
    const wx=i=>ox+(i%n)*s,wy=i=>oy+Math.floor(i/n)*s;
    const index=p=>Math.max(0,Math.min(n-1,Math.round((p.y-oy)/s)))*n+Math.max(0,Math.min(n-1,Math.round((p.x-ox)/s)));
    const snap=p=>{const i=index(p);return{x:wx(i),y:wy(i)}};
    for(let y=1;y<n-1;y++)for(let x=1;x<n-1;x++){
      const i=y*n+x;grade[i]=Math.hypot((z[i+1]-z[i-1])/(2*s),(z[i+n]-z[i-n])/(2*s));safe[i]=grade[i]<=maxSlope?1:0;
    }
    function collision(ax,ay,bx,by){
      const dx=bx-ax,dy=by-ay,len=dx*dx+dy*dy;
      return rocks.some(r=>{const t=len?Math.max(0,Math.min(1,((r.x-ax)*dx+(r.y-ay)*dy)/len)):0;return Math.hypot(ax+t*dx-r.x,ay+t*dy-r.y)<=r.radius*1.15+.65});
    }
    const allowed=safe.map((v,i)=>v&&!collision(wx(i),wy(i),wx(i),wy(i))?1:0);
    function connected(a,b){
      const from=index(a),to=index(b);if(!allowed[from]||!allowed[to])return false;
      const seen=new Uint8Array(z.length),queue=[from];seen[from]=1;
      for(let k=0;k<queue.length;k++){
        const u=queue[k];if(u===to)return true;const x=u%n,y=Math.floor(u/n);
        for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
          const nx=x+dx,ny=y+dy;if(!(dx||dy)||nx<0||ny<0||nx>=n||ny>=n)continue;
          const v=ny*n+nx;if(seen[v]||!allowed[v])continue;
          if(dx&&dy&&(!allowed[y*n+nx]||!allowed[ny*n+x]))continue;
          if(Math.abs(z[v]-z[u])/(s*Math.hypot(dx,dy))>maxSlope||collision(wx(u),wy(u),wx(v),wy(v)))continue;
          seen[v]=1;queue.push(v);
        }
      }return false;
    }
    function resolveColor(name){const probe=document.createElement('span');probe.hidden=true;probe.style.color='var('+name+')';root.append(probe);const c=getComputedStyle(probe).color;probe.remove();return c}
    function rgb(color){const c=document.createElement('canvas');c.width=c.height=1;const g=c.getContext('2d');g.fillStyle=color;g.fillRect(0,0,1,1);return g.getImageData(0,0,1,1).data}
    function theme(){
      colors={text:resolveColor('--foreground'),bg:resolveColor('--background'),grid:resolveColor('--border'),low:resolveColor('--muted'),high:resolveColor('--viz-series-1'),start:resolveColor('--green'),goal:resolveColor('--red')};
      const a=rgb(colors.low),b=rgb(colors.high);ramp=Array.from({length:64},(_,i)=>{const t=i/63*.65;return 'rgb('+[0,1,2].map(k=>Math.round(a[k]*(1-t)+b[k]*t)).join(',')+')'});draw();
    }
    function marker(p,label,diamond){
      const x=frame.left+(p.x-ox)/50*frame.side,y=frame.top+(25-p.y)/50*frame.side;
      ctx.beginPath();if(diamond){ctx.moveTo(x,y-12);ctx.lineTo(x+12,y);ctx.lineTo(x,y+12);ctx.lineTo(x-12,y);ctx.closePath()}else ctx.arc(x,y,11,0,Math.PI*2);
      ctx.fillStyle=colors.bg;ctx.fill();ctx.strokeStyle=diamond?colors.goal:colors.start;ctx.lineWidth=3;ctx.stroke();ctx.fillStyle=colors.text;ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(label,x,y);
    }
    function draw(){
      const rect=canvas.getBoundingClientRect();if(!rect.width)return;const dpr=window.devicePixelRatio||1,w=rect.width,h=rect.height;
      canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);
      const side=Math.min(w-62,h-58),left=42+(w-62-side)/2,top=12;frame={left,top,side};
      ctx.clearRect(0,0,w,h);ctx.save();ctx.beginPath();ctx.rect(left,top,side,side);ctx.clip();
      for(let i=0;i<z.length;i++){
        const x=left+(wx(i)-ox-s/2)/50*side,y=top+(25-wy(i)-s/2)/50*side,size=s/50*side;
        ctx.fillStyle=ramp[Math.max(0,Math.min(63,Math.round((z[i]-low)/(high-low)*63)))];ctx.fillRect(x,y,size+.5,size+.5);
        if(grade[i]>maxSlope){ctx.save();ctx.beginPath();ctx.rect(x,y,size,size);ctx.clip();ctx.strokeStyle=colors.goal;ctx.globalAlpha=.28;ctx.lineWidth=1;for(let j=-size;j<size*2;j+=6){ctx.beginPath();ctx.moveTo(x+j,y+size);ctx.lineTo(x+j+size,y);ctx.stroke()}ctx.restore()}
      }
      ctx.strokeStyle=colors.grid;ctx.lineWidth=1;for(let v=-25;v<=25;v+=10){const p=(v+25)/50*side;ctx.beginPath();ctx.moveTo(left+p,top);ctx.lineTo(left+p,top+side);ctx.moveTo(left,top+side-p);ctx.lineTo(left+side,top+side-p);ctx.stroke()}
      for(const r of rocks){ctx.beginPath();ctx.arc(left+(r.x-ox)/50*side,top+(25-r.y)/50*side,Math.max(3,r.radius/50*side),0,Math.PI*2);ctx.fillStyle=colors.text;ctx.fill();ctx.strokeStyle=colors.bg;ctx.lineWidth=1;ctx.stroke()}
      const base=parseFloat(getComputedStyle(root).getPropertyValue('--font-size-base'))||14;ctx.font='500 '+base+'px system-ui';marker(start,'S',false);marker(goal,'G',true);
      if(hover){const px=left+(hover.x-ox)/50*side,py=top+(25-hover.y)/50*side;ctx.strokeStyle=colors.text;ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(px-5,py);ctx.lineTo(px+5,py);ctx.moveTo(px,py-5);ctx.lineTo(px,py+5);ctx.stroke()}
      ctx.restore();ctx.font='400 '+Math.max(11,base-2)+'px system-ui';ctx.fillStyle=colors.text;ctx.textBaseline='middle';
      const ticks=w<400?[-25,0,25]:[-25,-15,-5,5,15,25];
      for(const v of ticks){const p=(v+25)/50*side;ctx.textAlign='center';ctx.fillText(String(v),left+p,top+side+16);ctx.textAlign='right';ctx.fillText(String(v),left-8,top+side-p)}
      ctx.textAlign='center';ctx.fillText('X (m)',left+side/2,top+side+37);ctx.textAlign='left';ctx.fillText('Y (m)',left-37,top-3);
    }
    function fmt(v){return Number(v.toFixed(6)).toString()}
    function update(){
      const selected=mode==='start'?start:goal;qx.value=fmt(selected.x);qy.value=fmt(selected.y);
      root.querySelectorAll('[data-mode]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.mode===mode)));
      root.querySelector('[data-coordinates]').textContent='S ('+fmt(start.x)+', '+fmt(start.y)+')  ·  G ('+fmt(goal.x)+', '+fmt(goal.y)+')  ·  '+Math.hypot(start.x-goal.x,start.y-goal.y).toFixed(2)+' m apart';
      const inBounds=[start.x,start.y,goal.x,goal.y].every(v=>Number.isFinite(v)&&v>=-23&&v<=23);
      valid=inBounds&&connected(start,goal)&&Math.hypot(start.x-goal.x,start.y-goal.y)>.3;
      status.textContent=!inBounds?'Choose coordinates between −23 and +23 m.':valid?'Current rock layout: connected grid route.':'No connected route for these points on the current layout; choose another location.';
      status.className=valid?'text-small':'text-small text-destructive';
      command.value='.\\generate-terrain.ps1 -StartX '+fmt(start.x)+' -StartY '+fmt(start.y)+' -GoalX '+fmt(goal.x)+' -GoalY '+fmt(goal.y)+' -RockCount '+rocks.length+' -Seed '+data.seed+' -TerrainProfile connected -MaxTerrainGrade '+data.maxTerrainGrade;
      apply.disabled=copy.disabled=!valid;apply.hidden=!window.openai?.sendFollowUpMessage;draw();
    }
    function save(){if(window.openai?.setWidgetState)window.openai.setWidgetState({modelContent:{start,goal},privateContent:{mode}}).catch(()=>{})}
    function restore(state){const content=state?.modelContent??state;if(content?.start&&content?.goal&&[content.start.x,content.start.y,content.goal.x,content.goal.y].every(Number.isFinite)){start={...content.start};goal={...content.goal}}if(state?.privateContent?.mode==='goal')mode='goal';update()}
    function pick(e){if(!frame)return null;const r=canvas.getBoundingClientRect(),px=e.clientX-r.left,py=e.clientY-r.top;if(px<frame.left||px>frame.left+frame.side||py<frame.top||py>frame.top+frame.side)return null;return snap({x:ox+(px-frame.left)/frame.side*50,y:25-(py-frame.top)/frame.side*50})}
    canvas.addEventListener('pointermove',e=>{hover=pick(e);if(hover){const i=index(hover);detail.textContent='X '+fmt(hover.x)+' m  ·  Y '+fmt(hover.y)+' m  ·  altitude '+z[i].toFixed(2)+' m  ·  slope '+(Math.atan(grade[i])*180/Math.PI).toFixed(1)+'°'+(!safe[i]?' — too steep':'')}draw()});
    canvas.addEventListener('pointerleave',()=>{hover=null;draw()});
    canvas.addEventListener('click',e=>{const p=pick(e);if(!p)return;if(mode==='start')start=p;else goal=p;update();save()});
    root.querySelectorAll('[data-mode]').forEach(b=>b.addEventListener('click',()=>{mode=b.dataset.mode;update();save()}));
    function manual(){const x=Number(qx.value),y=Number(qy.value);if(!Number.isFinite(x)||!Number.isFinite(y))return;const p=snap({x,y});if(mode==='start')start=p;else goal=p;update();save()}
    qx.addEventListener('change',manual);qy.addEventListener('change',manual);
    copy.addEventListener('click',async()=>{try{await navigator.clipboard.writeText(command.value);copyStatus.textContent='Copied'}catch{command.focus();command.select();copyStatus.textContent='Selected — press Ctrl+C'}});
    apply.addEventListener('click',async()=>{if(!valid||!window.openai?.sendFollowUpMessage)return;await window.openai.sendFollowUpMessage({title:'Apply selected rover positions',prompt:'Update the rover world using START X='+fmt(start.x)+', Y='+fmt(start.y)+' and GOAL X='+fmt(goal.x)+', Y='+fmt(goal.y)+'. Keep the connected slope-capped Moose terrain, working controller, speed and LIDAR panel. Regenerate with '+rocks.length+' rocks and seed '+data.seed+'. Check that all free grid cells and selected endpoints stay connected. If needed, try another rock seed without moving my selected endpoints. Do not access my screen.'})});
    root.querySelector('[data-low]').textContent=low.toFixed(1)+' m';root.querySelector('[data-high]').textContent=high.toFixed(1)+' m';
    new ResizeObserver(()=>draw()).observe(canvas);new MutationObserver(()=>theme()).observe(document.documentElement,{attributes:true,attributeFilter:['class','style','data-theme']});
    window.addEventListener('openai:set_globals',e=>{const state=e.detail?.globals?.widgetState;if(state)restore(state);theme()});
    theme();restore(window.openai?.widgetState);
  })();
  
