#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QRegularExpression>
#include <QLocale>
#include <QVector>
#include <QSet>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

const double PI = 3.14159265358979323846;

struct Pair { int codeLine; int valueLine; int code; QString value; };
struct Point { double x = 0.0; double y = 0.0; };

double dist(const Point &a, const Point &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

Point midpoint(const Point &a, const Point &b)
{
    return {(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
}

bool parseDouble(const QString &s, double &v)
{
    bool ok = false;
    v = QLocale::c().toDouble(s.trimmed(), &ok);
    return ok && std::isfinite(v);
}

QString fmt(double v)
{
    if (std::abs(v) < 5e-13) v = 0.0;
    return QString::number(v, 'g', 16);
}

struct Endpoint {
    Point p;
    int entity = -1;
    int side = 0; // 0=start, 1=end
};

struct Entity {
    QString type;
    int recordStart = -1;
    int recordEnd = -1;
    int xStartLine = -1, yStartLine = -1;
    int xEndLine = -1, yEndLine = -1;
    int centerXLine = -1, centerYLine = -1;
    int radiusLine = -1, angleStartLine = -1, angleEndLine = -1;
    Point start, end, center;
    double radius = 0.0, angleStart = 0.0, angleEnd = 0.0;
    Point arcMid;
    bool valid = false;
    bool closed = false;
    QVector<int> vertexXLines;
    QVector<int> vertexYLines;
    QVector<Point> vertices;
};

struct ParseResult {
    QStringList lines;
    QVector<Pair> pairs;
    QVector<Entity> entities;
    QStringList errors;
    QString newline = "\n";
    bool trailingNewline = true;
};

bool valueFor(const QVector<Pair> &pairs, int from, int to, int code,
              QString &value, int *valueLine = nullptr)
{
    for (int i = from; i < to; ++i) {
        if (pairs[i].code == code) {
            value = pairs[i].value;
            if (valueLine) *valueLine = pairs[i].valueLine;
            return true;
        }
    }
    return false;
}

bool numberFor(const QVector<Pair> &pairs, int from, int to, int code,
               double &value, int *valueLine = nullptr)
{
    QString s;
    int line = -1;
    if (!valueFor(pairs, from, to, code, s, &line) || !parseDouble(s, value))
        return false;
    if (valueLine) *valueLine = line;
    return true;
}

int nextRecord(const QVector<Pair> &p, int i)
{
    int j = i + 1;
    while (j < p.size() && p[j].code != 0) ++j;
    return j;
}

bool makeArcPoint(const Point &c, double r, double degrees, Point &p)
{
    const double a = degrees * PI / 180.0;
    p = {c.x + r * std::cos(a), c.y + r * std::sin(a)};
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool circleThrough3(const Point &a, const Point &b, const Point &c,
                    Point &center, double &radius)
{
    const double d = 2.0 * (a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y));
    if (std::abs(d) < 1e-10) return false;
    const double aa = a.x*a.x + a.y*a.y;
    const double bb = b.x*b.x + b.y*b.y;
    const double cc = c.x*c.x + c.y*c.y;
    center.x = (aa*(b.y-c.y) + bb*(c.y-a.y) + cc*(a.y-b.y)) / d;
    center.y = (aa*(c.x-b.x) + bb*(a.x-c.x) + cc*(b.x-a.x)) / d;
    radius = dist(center, a);
    return std::isfinite(radius) && radius > 1e-10;
}

double norm360(double a)
{
    a = std::fmod(a, 360.0);
    if (a < 0) a += 360.0;
    return a;
}

bool rebuildArc(Entity &e, const Point &newStart, const Point &newEnd,
                bool changeStart, bool changeEnd, QStringList &errors)
{
    Point s = changeStart ? newStart : e.start;
    Point t = changeEnd ? newEnd : e.end;
    Point c;
    double r = 0.0;
    if (!circleThrough3(s, e.arcMid, t, c, r)) {
        errors << "Un arco non e' stato corretto: i tre punti risultano quasi allineati.";
        return false;
    }
    double a1 = norm360(std::atan2(s.y-c.y, s.x-c.x) * 180.0 / PI);
    double a2 = norm360(std::atan2(t.y-c.y, t.x-c.x) * 180.0 / PI);
    // DXF ARC percorre l'arco in senso antiorario da angolo iniziale a finale.
    // Manteniamo il verso e scegliamo la soluzione che contiene il punto medio originale.
    double sweep = norm360(a2-a1);
    double am = norm360(std::atan2(e.arcMid.y-c.y, e.arcMid.x-c.x) * 180.0 / PI);
    double midSweep = norm360(am-a1);
    if (midSweep > sweep + 1e-5) {
        // La ricostruzione non conserva il tratto corretto: non tocchiamo l'arco.
        errors << "Un arco non e' stato corretto: il punto medio non appartiene al tratto ricostruito.";
        return false;
    }
    e.center = c; e.radius = r; e.angleStart = a1; e.angleEnd = a2;
    e.start = s; e.end = t;
    return true;
}

ParseResult parseDxf(const QByteArray &data)
{
    ParseResult out;
    QString text = QString::fromLatin1(data);
    out.newline = text.contains("\r\n") ? "\r\n" : "\n";
    out.trailingNewline = text.endsWith('\n');
    out.lines = text.split('\n');
    for (QString &line : out.lines)
        if (line.endsWith('\r')) line.chop(1);

    for (int i = 0; i + 1 < out.lines.size(); i += 2) {
        bool ok = false;
        int code = out.lines[i].trimmed().toInt(&ok);
        if (!ok) continue;
        out.pairs.append({i, i+1, code, out.lines[i+1].trimmed()});
    }

    bool inEntities = false;
    for (int i = 0; i < out.pairs.size();) {
        const Pair &p = out.pairs[i];
        if (p.code != 0) { ++i; continue; }
        if (p.value == "SECTION") {
            if (i+1 < out.pairs.size() && out.pairs[i+1].code == 2)
                inEntities = (out.pairs[i+1].value == "ENTITIES");
            ++i; continue;
        }
        if (p.value == "ENDSEC") { inEntities = false; ++i; continue; }
        if (!inEntities) { ++i; continue; }

        const int start = i;
        int end = nextRecord(out.pairs, i);
        QString type = p.value;
        Entity e; e.type = type; e.recordStart = start; e.recordEnd = end;

        if (type == "LINE") {
            bool ok = numberFor(out.pairs,start+1,end,10,e.start.x,&e.xStartLine) &&
                      numberFor(out.pairs,start+1,end,20,e.start.y,&e.yStartLine) &&
                      numberFor(out.pairs,start+1,end,11,e.end.x,&e.xEndLine) &&
                      numberFor(out.pairs,start+1,end,21,e.end.y,&e.yEndLine);
            e.valid = ok;
            if (!ok) out.errors << "LINE con coordinate mancanti o non valide.";
        } else if (type == "ARC") {
            bool ok = numberFor(out.pairs,start+1,end,10,e.center.x,&e.centerXLine) &&
                      numberFor(out.pairs,start+1,end,20,e.center.y,&e.centerYLine) &&
                      numberFor(out.pairs,start+1,end,40,e.radius,&e.radiusLine) &&
                      numberFor(out.pairs,start+1,end,50,e.angleStart,&e.angleStartLine) &&
                      numberFor(out.pairs,start+1,end,51,e.angleEnd,&e.angleEndLine) && e.radius > 0;
            if (ok) {
                makeArcPoint(e.center,e.radius,e.angleStart,e.start);
                makeArcPoint(e.center,e.radius,e.angleEnd,e.end);
                double sweep = norm360(e.angleEnd-e.angleStart);
                makeArcPoint(e.center,e.radius,e.angleStart+sweep*0.5,e.arcMid);
            }
            e.valid = ok;
            if (!ok) out.errors << "ARC con parametri mancanti o non validi.";
        } else if (type == "LWPOLYLINE") {
            double flags = 0;
            numberFor(out.pairs,start+1,end,70,flags);
            e.closed = (static_cast<int>(flags) & 1) != 0;
            for (int k=start+1;k<end;++k) {
                if (out.pairs[k].code == 10) {
                    double x=0; if (!parseDouble(out.pairs[k].value,x)) continue;
                    int yLine=-1; double y=0;
                    if (k+1<end && out.pairs[k+1].code==20 && parseDouble(out.pairs[k+1].value,y))
                        yLine=out.pairs[k+1].valueLine;
                    if (yLine>=0) {
                        e.vertices.append({x,y}); e.vertexXLines.append(out.pairs[k].valueLine); e.vertexYLines.append(yLine);
                    }
                }
            }
            e.valid = e.vertices.size() >= 2;
            if (e.valid) { e.start=e.vertices.first(); e.end=e.vertices.last(); }
            else out.errors << "LWPOLYLINE con vertici non validi.";
        } else if (type == "POLYLINE") {
            double flags=0; numberFor(out.pairs,start+1,end,70,flags);
            e.closed = (static_cast<int>(flags) & 1) != 0;
            int k=end;
            while (k<out.pairs.size() && out.pairs[k].code==0 && out.pairs[k].value=="VERTEX") {
                int ve=nextRecord(out.pairs,k);
                double x=0,y=0; int xl=-1,yl=-1;
                if (numberFor(out.pairs,k+1,ve,10,x,&xl) && numberFor(out.pairs,k+1,ve,20,y,&yl)) {
                    e.vertices.append({x,y}); e.vertexXLines.append(xl); e.vertexYLines.append(yl);
                }
                k=ve;
            }
            if (k<out.pairs.size() && out.pairs[k].code==0 && out.pairs[k].value=="SEQEND")
                e.recordEnd=nextRecord(out.pairs,k);
            e.valid=e.vertices.size()>=2;
            if (e.valid) { e.start=e.vertices.first(); e.end=e.vertices.last(); }
            else out.errors << "POLYLINE classica con vertici non validi.";
            end=e.recordEnd;
        }

        if ((type=="LINE" || type=="ARC" || type=="LWPOLYLINE" || type=="POLYLINE") && e.valid)
            out.entities.append(e);
        i = std::max(end, i+1);
    }
    return out;
}

struct DSU {
    QVector<int> p;
    explicit DSU(int n):p(n){ for(int i=0;i<n;++i)p[i]=i; }
    int find(int x){ return p[x]==x?x:p[x]=find(p[x]); }
    void unite(int a,int b){ a=find(a);b=find(b);if(a!=b)p[b]=a; }
};

struct ProcessResult {
    QByteArray output;
    int lineCount=0, arcCount=0, lwCount=0, polyCount=0;
    int changedGroups=0, changedEndpoints=0, ignoredPairs=0;
    QStringList messages;
};

// Soglia per riconoscere un collegamento gia' esistente. E' espressa in
// unita' macchina del double (64 bit), adattata alla scala delle coordinate.
double coincidenceEpsilon(const Point &a, const Point &b)
{
    const double scale = std::max({1.0, std::abs(a.x), std::abs(a.y),
                                   std::abs(b.x), std::abs(b.y)});
    return 64.0 * std::numeric_limits<double>::epsilon() * scale;
}

ProcessResult processDxf(const QByteArray &data, double tolerance)
{
    ProcessResult result;
    ParseResult parsed=parseDxf(data);
    result.messages=parsed.errors;
    QVector<Endpoint> ends;
    for (int i=0;i<parsed.entities.size();++i) {
        const Entity &e=parsed.entities[i];
        if(e.type=="LINE") result.lineCount++;
        if(e.type=="ARC") result.arcCount++;
        if(e.type=="LWPOLYLINE") result.lwCount++;
        if(e.type=="POLYLINE") result.polyCount++;
        if(e.closed) continue;
        ends.append({e.start,i,0});
        ends.append({e.end,i,1});
    }

    // PRIMO PASSAGGIO: individua i collegamenti gia' presenti alla precisione
    // numerica del double. Gli estremi che partecipano a un collegamento
    // esistente vengono protetti e non saranno candidati a nuove unioni.
    QVector<bool> alreadyConnected(ends.size(), false);
    for(int i=0;i<ends.size();++i) {
        for(int j=i+1;j<ends.size();++j) {
            if(ends[i].entity==ends[j].entity) continue;
            const double eps=coincidenceEpsilon(ends[i].p,ends[j].p);
            if(dist(ends[i].p,ends[j].p)<=eps) {
                alreadyConnected[i]=true;
                alreadyConnected[j]=true;
            }
        }
    }

    // SECONDO PASSAGGIO: cerca nuove unioni soltanto tra estremi non gia'
    // collegati. In questo modo un piccolo ARC che collega gia' due LINE non
    // viene scavalcato unendo direttamente le due LINE tra loro.
    DSU dsu(ends.size());
    for(int i=0;i<ends.size();++i) {
        if(alreadyConnected[i]) continue;
        for(int j=i+1;j<ends.size();++j) {
            if(alreadyConnected[j]) continue;
            if(ends[i].entity==ends[j].entity) continue;
            if(dist(ends[i].p,ends[j].p)>=tolerance) {
                result.ignoredPairs++;
                continue;
            }
            int ri=dsu.find(i), rj=dsu.find(j);
            if(ri==rj) continue;
            QVector<int> gi,gj;
            for(int k=0;k<ends.size();++k) {
                if(dsu.find(k)==ri) gi.append(k);
                if(dsu.find(k)==rj) gj.append(k);
            }
            bool compatible=true;
            for(int a:gi) for(int b:gj) {
                    if(ends[a].entity==ends[b].entity ||
                        dist(ends[a].p,ends[b].p)>=tolerance)
                        compatible=false;
                }
            if(compatible) dsu.unite(ri,rj);
        }
    }

    QVector<Point> newPoints;
    newPoints.reserve(ends.size());
    for(const Endpoint &ep:ends) newPoints.append(ep.p);
    QSet<int> handled;
    for(int i=0;i<ends.size();++i) {
        if(alreadyConnected[i]) continue;
        int root=dsu.find(i);
        if(handled.contains(root)) continue;
        handled.insert(root);
        QVector<int> members;
        for(int j=0;j<ends.size();++j)
            if(!alreadyConnected[j] && dsu.find(j)==root) members.append(j);
        if(members.size()<2) continue;
        // Le coordinate di LINE e polilinee fanno da riferimento per gli ARC;
        // se il gruppo contiene solo archi, si usa la media degli estremi.
        Point target{0,0};
        int count=0;
        for(int m:members) {
            const QString t=parsed.entities[ends[m].entity].type;
            if(t!="ARC") {
                target.x+=ends[m].p.x;
                target.y+=ends[m].p.y;
                ++count;
            }
        }
        if(count==0) {
            for(int m:members) {
                target.x+=ends[m].p.x;
                target.y+=ends[m].p.y;
                ++count;
            }
        }
        target.x/=count;
        target.y/=count;
        bool any=false;
        for(int m:members) {
            if(dist(newPoints[m],target)>coincidenceEpsilon(newPoints[m],target)) {
                newPoints[m]=target;
                any=true;
            }
        }
        if(any) {
            result.changedGroups++;
            result.changedEndpoints+=members.size();
        }
    }

    // Applica le coordinate alle entita', ricostruendo gli ARC modificati.
    for(int ei=0;ei<parsed.entities.size();++ei) {
        Entity &e=parsed.entities[ei];
        int si=-1,ti=-1;
        for(int k=0;k<ends.size();++k) if(ends[k].entity==ei) { if(ends[k].side==0)si=k;else ti=k; }
        if(si<0||ti<0) continue;
        Point ns=newPoints[si], nt=newPoints[ti];
        bool cs=dist(ns,e.start)>coincidenceEpsilon(ns,e.start);
        bool ct=dist(nt,e.end)>coincidenceEpsilon(nt,e.end);
        if(!cs&&!ct) continue;
        if(e.type=="ARC") {
            if(!rebuildArc(e,ns,nt,cs,ct,result.messages)) continue;
            parsed.lines[e.centerXLine]=fmt(e.center.x);
            parsed.lines[e.centerYLine]=fmt(e.center.y);
            parsed.lines[e.radiusLine]=fmt(e.radius);
            parsed.lines[e.angleStartLine]=fmt(e.angleStart);
            parsed.lines[e.angleEndLine]=fmt(e.angleEnd);
        } else if(e.type=="LINE") {
            parsed.lines[e.xStartLine]=fmt(ns.x); parsed.lines[e.yStartLine]=fmt(ns.y);
            parsed.lines[e.xEndLine]=fmt(nt.x); parsed.lines[e.yEndLine]=fmt(nt.y);
        } else if(!e.closed && e.vertices.size()>=2) {
            parsed.lines[e.vertexXLines.first()]=fmt(ns.x);
            parsed.lines[e.vertexYLines.first()]=fmt(ns.y);
            parsed.lines[e.vertexXLines.last()]=fmt(nt.x);
            parsed.lines[e.vertexYLines.last()]=fmt(nt.y);
        }
    }
    QString output=parsed.lines.join(parsed.newline);
    if(parsed.trailingNewline && !output.endsWith(parsed.newline)) output+=parsed.newline;
    result.output=output.toLatin1();
    return result;
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->txtMessaggi->setReadOnly(true);
    ui->btnSalva->setEnabled(false);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_btnApri_clicked()
{
    const QString name=QFileDialog::getOpenFileName(this,"Apri file DXF",QString(),"File DXF (*.dxf);;Tutti i file (*)");
    if(name.isEmpty()) return;
    QFile f(name);
    if(!f.open(QIODevice::ReadOnly)) { QMessageBox::critical(this,"Errore","Impossibile aprire il file DXF."); return; }
    dxfData=f.readAll(); f.close(); filePath=name; fileLoaded=true; processed=false; processedDxfData.clear();
    ui->lblFile->setText(QFileInfo(filePath).fileName());
    ui->btnSalva->setEnabled(false);
    ui->txtMessaggi->clear();
    ui->txtMessaggi->append("File DXF caricato correttamente.");
    ui->txtMessaggi->append("Dimensione: "+QString::number(dxfData.size())+" byte");
}

void MainWindow::on_btnElabora_clicked()
{
    if(!fileLoaded) { QMessageBox::warning(this,"Attenzione","Prima devi aprire un file DXF."); return; }
    bool ok=false;
    const double tol=QLocale::c().toDouble(ui->editTolleranza->text().trimmed(),&ok);
    if(!ok || !std::isfinite(tol) || tol<=0) { QMessageBox::warning(this,"Tolleranza non valida","Inserisci una tolleranza numerica positiva, in millimetri."); return; }
    ProcessResult r=processDxf(dxfData,tol);
    processedDxfData=r.output; processed=true; ui->btnSalva->setEnabled(true);
    ui->txtMessaggi->clear();
    ui->txtMessaggi->append("ANALISI E CORREZIONE DXF");
    ui->txtMessaggi->append("--------------------------");
    ui->txtMessaggi->append("LINE: "+QString::number(r.lineCount));
    ui->txtMessaggi->append("ARC: "+QString::number(r.arcCount));
    ui->txtMessaggi->append("LWPOLYLINE: "+QString::number(r.lwCount));
    ui->txtMessaggi->append("POLYLINE: "+QString::number(r.polyCount));
    ui->txtMessaggi->append("Tolleranza: "+QString::number(tol,'g',8)+" mm");
    ui->txtMessaggi->append("Gruppi di estremi corretti: "+QString::number(r.changedGroups));
    ui->txtMessaggi->append("Estremi coinvolti: "+QString::number(r.changedEndpoints));
    for(const QString &m:r.messages) ui->txtMessaggi->append("AVVISO: "+m);
    ui->txtMessaggi->append("");
    ui->txtMessaggi->append("Elaborazione completata. Il file originale e' rimasto invariato.");
    ui->txtMessaggi->append("Premi 'Salva file' per creare il DXF corretto.");
}

void MainWindow::on_btnSalva_clicked()
{
    if(!processed) { QMessageBox::warning(this,"Attenzione","Prima devi elaborare il DXF."); return; }
    QString suggested=QFileInfo(filePath).path()+"/"+QFileInfo(filePath).completeBaseName()+"_corretto.dxf";
    QString name=QFileDialog::getSaveFileName(this,"Salva DXF corretto",suggested,"File DXF (*.dxf)");
    if(name.isEmpty()) return;
    if(QFileInfo(name).suffix().isEmpty()) name+=".dxf";
    QFile f(name);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Truncate)) { QMessageBox::critical(this,"Errore","Impossibile salvare il file DXF."); return; }
    if(f.write(processedDxfData)!=processedDxfData.size()) { f.close(); QMessageBox::critical(this,"Errore","Scrittura del file incompleta."); return; }
    f.close();
    ui->txtMessaggi->append("File salvato: "+name);
    QMessageBox::information(this,"Salvataggio completato","DXF corretto salvato correttamente.");
}
