; Example friction surfacing job — annotated G-code
;
; Directives handled by the job runner (never sent to Klipper):
;   ;FORCE=<units>             set Z-force control target
;   ;WAIT_FORCE TIMEOUT=<s>    wait until force is within deadband of target
;   ;DWELL=<s>                 hold N seconds while force control regulates
;
; Touch-down / dwell / retract cycle:

;FORCE=0
G1 Z45 F600          ; travel at safe height
G1 X25 Y30 F600      ; over the spot
G1 Z10 F300          ; park just above the surface

;FORCE=800
;WAIT_FORCE TIMEOUT=30
;DWELL=10

;FORCE=0
G91
G1 Z5 F600           ; lift off
G90
G1 Z45 F600          ; back to safe height
