UNISCI DXF - prima versione

Progetto Qt Widgets / qmake, C++17. L'interfaccia originale e' mantenuta.

Funzioni implementate:
- lettura delle entita' nella sezione ENTITIES;
- riconoscimento di LINE, ARC, LWPOLYLINE e POLYLINE classiche;
- individuazione degli estremi aperti e confronto tra entita' diverse;
- raggruppamento prudente: nessun gruppo puo' contenere due estremi della stessa entita' e il diametro del gruppo resta sotto la tolleranza;
- correzione delle LINE e dei vertici terminali delle polilinee;
- ricostruzione geometrica degli ARC modificati usando i due estremi e il punto medio originale;
- salvataggio in un nuovo DXF, senza sovrascrivere automaticamente l'originale.

Limiti da tenere presenti:
- elaborazione 2D XY; non e' un validatore completo di ogni variante DXF;
- le polilinee chiuse non vengono aperte o collegate dall'esterno;
- i vertici interni delle polilinee non vengono spostati;
- i bulge delle polilinee sono mantenuti invariati;
- entita' in BLOCKS, INSERT, SPLINE, ELLIPSE, REGION e altre entita' non sono riparate;
- se un ARC non puo' essere ricostruito in modo stabile, viene lasciato invariato e viene mostrato un avviso.

Test:
Aprire test_polilinee.dxf. Contiene LINE, LWPOLYLINE aperta e chiusa,
POLYLINE classica e ARC. Il test serve a controllare il riconoscimento;
non costituisce una certificazione per tutti i DXF/CAM.

Nota: in questo ambiente non e' disponibile qmake/Qt, quindi il progetto non e'
stato compilato qui. Compilare in Qt Creator prima di usare il programma su
file di produzione e controllare il DXF salvato in LibreCAD/CAM.
