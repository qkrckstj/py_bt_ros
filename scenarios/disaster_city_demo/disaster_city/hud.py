"""ROS 2 progress display. Each message is the public observation JSON."""
import json
import tkinter as tk
from tkinter import ttk

names=dict(fire='화재 진압',power_box='배전함 복구',hydrant='소화전 수리',small_debris='작은 잔해',large_debris='큰 잔해',fallen_tree='가로수',cordon='위험구역 통제')

def rows(obs):
    out=[]
    for key,j in obs['tasks'].items():
        value=max(0,min(100,100*j['progress_s']/j['seconds']))
        state={'pending':'집결 대기','working':'작업 중','paused':'중단','done':'완료'}[j['state']]
        out.append((str(key),names[j['kind']],state,j['working'],j['minimum'],value))
    return out

def main():
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node=rclpy.create_node('city_task_display')
    root=tk.Tk()
    root.title('도시 재난 · 작업 진행')
    title=ttk.Label(root,text='임무 상태 수신 대기',padding=12)
    title.grid(columnspan=3,sticky='w')
    widgets={}
    robot_text=ttk.Label(root,text='',padding=12,justify='left',font=('DejaVu Sans',10))
    robot_text.grid(row=20,columnspan=3,sticky='w')
    def update(msg):
        obs=json.loads(msg.data)
        data=rows(obs)
        if set(widgets)-{r[0] for r in data}:
            for group in widgets.values():
                for w in group: w.destroy()
            widgets.clear()
        title.configure(text=f"{obs['phase']}  ·  {obs['time_s']:.0f}초  ·  조사 {len(obs['surveyed'])}/{len(obs['survey'])}  ·  발견 {len(data)}개\n규칙 기반 할당 — PPO 모델 미학습")
        robot_text.configure(text='\n'.join(f"{r['name']}   배터리 {r['battery']:5.1f}%   {r['role']:8s} / {r['state']:10s}" for r in obs['robots']))
        for key,name,state,crew,minimum,value in data:
            if key not in widgets:
                group=(ttk.Label(root,padding=6),ttk.Progressbar(root,length=170,maximum=100),ttk.Label(root,width=7))
                for i,w in enumerate(group): w.grid(row=len(widgets)+1,column=i,sticky='w',padx=5,pady=3)
                widgets[key]=group
            label,bar,pct=widgets[key]
            label.configure(text=f'{name}   {state}   참여 {crew}대 / 최소 {minimum}대')
            bar.configure(value=value)
            pct.configure(text=f'{value:.0f}%')
    node.create_subscription(String,'/disaster/observation',update,10)
    def poll():
        rclpy.spin_once(node,timeout_sec=0.0)
        root.after(30,poll)
    root.after(0,poll)
    try: root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
