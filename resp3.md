Cosa ho scoperto e fatto
1. Il detector attivo è RealSense (C++), non quello che avevo strumentato. Lo standard (il robot a 44Hz) gira realsense_detector.cpp; la sentry usa la ZED. Per questo ho portato i fix sul detector C++ giusto.

2. Il trtexec ha individuato il vero collo di bottiglia:

Engine a 960×960, GPU compute 15.1 ms → tetto 66fps solo per l'inferenza; +8ms CPU seriali → 44Hz. Quadra.
Clock GPU 0.918 GHz → probabilmente non sei in max power.
I due leveroni più grossi sono camera-independent e a rischio-codice ZERO — falli tu adesso:

Ri-esporta il modello a 640×640 (ora 960): inferenza ~15→7ms → ~110fps di tetto. Il più grosso.
sudo nvpmodel -m 0 && sudo jetson_clocks: ~-25% sull'inferenza, gratis.
Rifai trtexec dopo: l'inferenza dovrebbe crollare a ~5-7ms.

3. Applicato sul detector RealSense (buildato OK): JSON opzionale (publish_keypoint_json: false — il topic non ha nessun subscriber, era CPU sprecata ogni frame) + latency ledger che logga LATENCY[ms] gpu=.. decode=.. pub=.. | loop=.. (..Hz) ogni ~90 frame. Stessi fix + pipeline sulla ZED (sentry).

Stadio 2 (MEM.GPU): scartato — salva 0.9ms su 15ms e rischia un crash cross-CUDA-context. Stadio 3 (pipeline) sul C++: differito apposta finché non leggi il ledger: tolto il JSON, se decode+pub è piccolo la pipeline rende poco, e un refactor CUDA non testabile sul detector di gara è troppo rischioso. Misura prima, poi decidiamo.

4. Pitch-lento + precedenza-facce: stessa radice, NON è dz. Dai dati: cmd_pitch salta 17.7 mrad/frame ma il servo fa 11 → pitch_locked solo 66%. E selected_face == best_face al 100% (la selezione funziona, e incoming_lookahead è usato). Quindi è la stessa command-jumpiness dello yaw, già mirata dai fix §4f B/C/D (non ancora testati), non un bug di selezione né un problema di altezze armature (Δz facce = 1cm).

Il tuo prossimo passo (decisivo)
640 re-export + jetson_clocks (i veri leveroni del framerate).
Lancia lo standard e mandami la riga LATENCY[ms] ... loop .. Hz del detector RealSense → da lì decidiamo se la pipeline C++ vale il rischio.
Testa i fix §4f B/C/D (sticky center-aim ecc.) — dovrebbero stabilizzare lock yaw e pitch.
Tutto in WORKLOG §4f.8/§4f.9 e in memoria. Quando hai i numeri del ledger + un bag nuovo coi fix attivi, chiudiamo framerate e lock insieme.

