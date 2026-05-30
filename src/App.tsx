import { useEffect, useMemo, useState } from "react";
import "./App.css";

type BearingMode = "F-18" | "T-45";

export default function App() {
  const centerX = 330;
  const centerY = 290;

  const SPRITE_OFFSET = 90;
  const BASE_PIXELS_PER_NM = 100;

  const leadKIAS = 250;
  const bankAngle = 30;
  const g = 32.174;

  const [bearingMode, setBearingMode] = useState<BearingMode>("F-18");
  const [leadAltitude, setLeadAltitude] = useState(10000);
  const [wingKIAS, setWingKIAS] = useState(250);
  const [spawnNm, setSpawnNm] = useState(1.5);
  const [timeScale, setTimeScale] = useState(1);
  const [paused, setPaused] = useState(false);

  const [orbitAngle, setOrbitAngle] = useState(0);
  const [rangeNm, setRangeNm] = useState(spawnNm);

  const bearingLineDeg = bearingMode === "F-18" ? 45 : 30;

  const deg = (d: number) => (d * Math.PI) / 180;
  const radToDeg = (r: number) => (r * 180) / Math.PI;

  function normalizeAngle(a: number) {
    let x = a;
    while (x > 180) x -= 360;
    while (x < -180) x += 360;
    return x;
  }

  function headingFromVector(x: number, y: number) {
    return radToDeg(Math.atan2(y, x));
  }

  // =========================
  // AIRSPEED / TURN PHYSICS
  // =========================

  const leadKTAS = leadKIAS * (1 + (leadAltitude / 1000) * 0.02);
  const wingKTAS = wingKIAS * (1 + (leadAltitude / 1000) * 0.02);

  const leadSpeedFps = leadKTAS * 1.68781;

  const turnRadiusFt =
    (leadSpeedFps * leadSpeedFps) / (g * Math.tan(deg(bankAngle)));

  const turnRadiusNm = turnRadiusFt / 6076.12;

  const turnRateRadPerHour = leadKTAS / turnRadiusNm;
  const turnRateRadPerSecond = turnRateRadPerHour / 3600;
  const turnRateDegPerSecond = radToDeg(turnRateRadPerSecond);

  const orbitPeriodSeconds = 360 / turnRateDegPerSecond;

  // Circle continues to shrink / expand as altitude changes.
  const pixelsPerNm = Math.min(
    BASE_PIXELS_PER_NM,
    180 / Math.max(turnRadiusNm, 1)
  );

  const orbitRadiusPx = turnRadiusNm * pixelsPerNm;

  // =========================
  // LEAD POSITION / HEADING
  // =========================

  const leadRad = deg(orbitAngle);

  const leadX = centerX + orbitRadiusPx * Math.cos(leadRad);
  const leadY = centerY + orbitRadiusPx * Math.sin(leadRad);

  // Left-hand turn.
  const leadHeading = radToDeg(leadRad) - 90;

  const leftWingHeading = leadHeading - 90;

  // Bearing line is aft of lead's left wingline.
  const bearingHeading = leftWingHeading - bearingLineDeg;

  // Wing-to-lead line of sight.
  const lineOfSightHeading = bearingHeading + 180;

  // =========================
  // MATH MODEL
  // =========================

  function solveGeometry(range: number) {
    const safeRange = Math.max(range, 0.0001);

    const rangePx = safeRange * pixelsPerNm;
    const bearingRad = deg(bearingHeading);

    const wingX = leadX + Math.cos(bearingRad) * rangePx;
    const wingY = leadY + Math.sin(bearingRad) * rangePx;

    // Lead-relative math frame:
    // +X = lead heading
    // +Y = lead's left wing
    const xRel = -safeRange * Math.sin(deg(bearingLineDeg));
    const yRel = safeRange * Math.cos(deg(bearingLineDeg));

    const wingPostX = xRel;
    const wingPostY = yRel - turnRadiusNm;

    // VRAD:
    // velocity required to remain fixed on the rotating bearing line
    // with zero closure.
    const vradX = -wingPostY * turnRateRadPerHour;
    const vradY = wingPostX * turnRateRadPerHour;
    const vradKt = Math.hypot(vradX, vradY);

    const toLeadX = -xRel / safeRange;
    const toLeadY = -yRel / safeRange;

    // |VRAD + C * toLead| = wingKTAS
    const dot = vradX * toLeadX + vradY * toLeadY;

    const discriminant =
      dot * dot - (vradKt * vradKt - wingKTAS * wingKTAS);

    let closureKt = 0;
    let canClose = true;

    if (range <= 0.0001) {
      closureKt = 0;
      canClose = true;
    } else if (discriminant < 0) {
      closureKt = 0;
      canClose = false;
    } else {
      const root = Math.sqrt(discriminant);

      const c1 = -dot + root;
      const c2 = -dot - root;

      const positive = [c1, c2].filter((c) => c > 0);

      if (positive.length === 0) {
        closureKt = 0;
        canClose = false;
      } else {
        closureKt = Math.min(...positive);
      }
    }

    const wingVelX = vradX + closureKt * toLeadX;
    const wingVelY = vradY + closureKt * toLeadY;

    const wingHeadingMath = headingFromVector(wingVelX, wingVelY);
    const misalignment = normalizeAngle(wingHeadingMath);

    const wingHeadingScreen = leadHeading - misalignment;

    // Signed ATA:
    // Negative = nose to the right side of bearing / LOS line.
    // Positive = nose to the left side of bearing / LOS line.
    const antennaTrainAngle = normalizeAngle(
      lineOfSightHeading - wingHeadingScreen
    );

    return {
      wingX,
      wingY,
      vradKt,
      closureKt,
      canClose,
      wingHeadingScreen,
      misalignment,
      antennaTrainAngle,
    };
  }

  function estimateSecondsToJoin() {
    let r = spawnNm;
    let t = 0;
    const dt = 0.25;

    for (let i = 0; i < 60000; i++) {
      const geo = solveGeometry(r);

      if (!geo.canClose || geo.closureKt <= 0) {
        return null;
      }

      r -= (geo.closureKt / 3600) * dt;
      t += dt;

      if (r <= 0.01) {
        return t;
      }
    }

    return null;
  }

  function resetJoin() {
    setRangeNm(spawnNm);
    setPaused(false);
  }

  // =========================
  // SIM LOOP
  // =========================

  useEffect(() => {
    const interval = setInterval(() => {
      if (paused) return;

      const dt = 0.03;

      setOrbitAngle((a) => {
        return (a - turnRateDegPerSecond * dt * timeScale) % 360;
      });

      setRangeNm((r) => {
        if (r <= 0) return 0;

        const geo = solveGeometry(r);

        if (!geo.canClose || geo.closureKt <= 0) {
          return r;
        }

        return Math.max(0, r - (geo.closureKt / 3600) * dt * timeScale);
      });
    }, 30);

    return () => clearInterval(interval);
  }, [
    paused,
    turnRateDegPerSecond,
    timeScale,
    wingKIAS,
    leadAltitude,
    orbitAngle,
    leadHeading,
    bearingHeading,
    lineOfSightHeading,
    pixelsPerNm,
    bearingLineDeg,
  ]);

  const currentGeo = solveGeometry(rangeNm);

  const estimatedJoinSeconds = useMemo(
    () => estimateSecondsToJoin(),
    [
      spawnNm,
      wingKIAS,
      leadAltitude,
      orbitAngle,
      leadHeading,
      bearingHeading,
      bearingLineDeg,
    ]
  );

  const tableRanges = useMemo(() => {
    const values: number[] = [];
    let r = Math.round(spawnNm * 4) / 4;

    while (r > 0) {
      values.push(Number(r.toFixed(2)));
      r -= 0.25;
    }

    values.push(0);
    return values;
  }, [spawnNm]);

  const wingX = currentGeo.wingX;
  const wingY = currentGeo.wingY;
  const wingHeading = currentGeo.wingHeadingScreen;

  //const joined = rangeNm <= 0.01;

  // =========================
  // SVG GEOMETRY
  // =========================

  const lineWeight = 2;
  const vectorLength = 115;

  const wingNoseX =
    wingX + Math.cos(deg(wingHeading)) * vectorLength;

  const wingNoseY =
    wingY + Math.sin(deg(wingHeading)) * vectorLength;

  const leadHeadingX = leadX + Math.cos(deg(leadHeading)) * 95;
  const leadHeadingY = leadY + Math.sin(deg(leadHeading)) * 95;

  const bearingUx = Math.cos(deg(bearingHeading));
  const bearingUy = Math.sin(deg(bearingHeading));

  const leadToCenterX = leadX - centerX;
  const leadToCenterY = leadY - centerY;

  const b =
    2 * (leadToCenterX * bearingUx + leadToCenterY * bearingUy);

  const c =
    leadToCenterX * leadToCenterX +
    leadToCenterY * leadToCenterY -
    orbitRadiusPx * orbitRadiusPx;

  const chordDisc = Math.max(0, b * b - 4 * c);
  const chordRoot = Math.sqrt(chordDisc);

  const t1 = (-b - chordRoot) / 2;
  const t2 = (-b + chordRoot) / 2;

  const tMin = Math.min(t1, t2);
  const tMax = Math.max(t1, t2);

  const chordLength = tMax - tMin;
  const chordExtension = chordLength * 0.25;

  const bearingLineStartX =
    leadX + bearingUx * (tMin - chordExtension);

  const bearingLineStartY =
    leadY + bearingUy * (tMin - chordExtension);

  const bearingLineEndX =
    leadX + bearingUx * (tMax + chordExtension);

  const bearingLineEndY =
    leadY + bearingUy * (tMax + chordExtension);

  const bearingLabelX = bearingLineEndX + 5;
  const bearingLabelY = bearingLineEndY + 5;

  const radiusLabelX = centerX + 14;
  const radiusLabelY = centerY - 14;

  const arcRadius = 42;

  const arcStart = deg(lineOfSightHeading);
  const arcEnd = deg(wingHeading);

  const arcStartX = wingX + Math.cos(arcStart) * arcRadius;
  const arcStartY = wingY + Math.sin(arcStart) * arcRadius;

  const arcEndX = wingX + Math.cos(arcEnd) * arcRadius;
  const arcEndY = wingY + Math.sin(arcEnd) * arcRadius;

  const ataSweep = normalizeAngle(wingHeading - lineOfSightHeading);
  const ataLargeArc = Math.abs(ataSweep) > 180 ? 1 : 0;
  const ataSweepFlag = ataSweep >= 0 ? 1 : 0;

  const ataArcPath = `
    M ${arcStartX} ${arcStartY}
    A ${arcRadius} ${arcRadius} 0 ${ataLargeArc} ${ataSweepFlag}
      ${arcEndX} ${arcEndY}
  `;

  const ataLabelOffsetHeading = wingHeading - 90;

  const ataDegreesLabelX =
    wingNoseX + Math.cos(deg(ataLabelOffsetHeading)) * 12;

  const ataDegreesLabelY =
    wingNoseY + Math.sin(deg(ataLabelOffsetHeading)) * 12;

  // RNG label behind the wingman's right stabilator.
  const rngAftOffset = 58;
  const rngRightOffset = 30;

  const rangeLabelX =
    wingX +
    Math.cos(deg(wingHeading + 180)) * rngAftOffset +
    Math.cos(deg(wingHeading + 90)) * rngRightOffset;

  const rangeLabelY =
    wingY +
    Math.sin(deg(wingHeading + 180)) * rngAftOffset +
    Math.sin(deg(wingHeading + 90)) * rngRightOffset;

  const vcLabelX = rangeLabelX;
  const vcLabelY = rangeLabelY + 18;

  // =========================
  // STYLES
  // =========================

  const pageStyle: React.CSSProperties = {
    minHeight: "100vh",
    background: "#111827",
    color: "white",
    fontFamily: "Arial, sans-serif",
    padding: "18px 28px",
    boxSizing: "border-box",
  };

  const titleStyle: React.CSSProperties = {
    textAlign: "center",
    fontSize: 42,
    fontWeight: 400,
    margin: "0 0 12px 0",
  };

  const topGridStyle: React.CSSProperties = {
    display: "grid",
    gridTemplateColumns: "1fr 1fr",
    gap: 90,
    maxWidth: 1040,
    margin: "0 auto 20px auto",
  };

  const panelTitleStyle: React.CSSProperties = {
    fontSize: 26,
    fontWeight: 700,
    textDecoration: "underline",
    textAlign: "center",
    marginBottom: 28,
  };

  const sectionHeaderStyle: React.CSSProperties = {
    fontSize: 23,
    textDecoration: "underline",
    margin: "18px 0 4px 0",
  };

  const textStyle: React.CSSProperties = {
    fontSize: 22,
    lineHeight: 1.35,
  };

  const sliderStyle: React.CSSProperties = {
    width: "100%",
    maxWidth: 330,
    margin: "4px 0 10px 0",
  };

  const buttonStyle = (active: boolean): React.CSSProperties => ({
    padding: "9px 14px",
    borderRadius: 8,
    border: active ? "2px solid #22c55e" : "1px solid #64748b",
    background: active ? "#163a23" : "#1f2937",
    color: "white",
    cursor: "pointer",
    fontSize: 15,
    marginRight: 8,
    marginTop: 6,
  });

  const simControlStyle: React.CSSProperties = {
    maxWidth: 660,
    margin: "8px auto 10px auto",
    textAlign: "center",
  };

  const simButtonStyle: React.CSSProperties = {
    padding: "8px 14px",
    borderRadius: 8,
    border: "1px solid #64748b",
    background: "#1f2937",
    color: "white",
    cursor: "pointer",
    fontSize: 15,
    margin: "8px 6px 0 6px",
  };

  const tableStyle: React.CSSProperties = {
    margin: "20px auto 24px auto",
    borderCollapse: "collapse",
    minWidth: 540,
    fontSize: 22,
  };

  const cellStyle: React.CSSProperties = {
    border: "1px solid rgba(255,255,255,0.75)",
    padding: "7px 12px",
    textAlign: "left",
  };

  const graphicWrapStyle: React.CSSProperties = {
    width: 660,
    height: 500,
    margin: "0 auto",
    background: "#101827",
    boxShadow: "8px 8px 18px rgba(0,0,0,0.35)",
    borderRadius: 4,
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
  };

  // =========================
  // RENDER
  // =========================

  return (
    <div style={pageStyle}>
      <h1 style={titleStyle}>Rendezvous Geometry Visualizer</h1>

      <div style={topGridStyle}>
        {/* LEAD COLUMN */}
        <div>
          <div style={panelTitleStyle}>Lead Aircraft</div>

          <div style={sectionHeaderStyle}>Editable Parameters</div>
          <div style={textStyle}>Rendezvous Altitude:</div>
          <input
            style={sliderStyle}
            type="range"
            min="0"
            max="40000"
            step="500"
            value={leadAltitude}
            onChange={(e) => setLeadAltitude(Number(e.target.value))}
          />
          <div style={textStyle}>{leadAltitude.toLocaleString()} ft</div>

          <div style={sectionHeaderStyle}>Fixed Parameters</div>
          <div style={textStyle}>Airspeed (KIAS): {leadKIAS}</div>
          <div style={textStyle}>Angle of Bank: {bankAngle}°</div>

          <div style={sectionHeaderStyle}>Dynamic Parameters</div>
          <div style={textStyle}>Airspeed (KTAS): {Math.round(leadKTAS)}</div>
          <div style={textStyle}>Turn Radius: {turnRadiusNm.toFixed(2)} nm</div>
          <div style={textStyle}>
            Time for 360° Turn: {orbitPeriodSeconds.toFixed(0)} sec
          </div>
        </div>

        {/* WING COLUMN */}
        <div>
          <div style={panelTitleStyle}>Wing Aircraft</div>

          <div style={sectionHeaderStyle}>Editable Parameters</div>

          <div style={textStyle}>Airspeed (KIAS): {wingKIAS}</div>
          <input
            style={sliderStyle}
            type="range"
            min="180"
            max="320"
            value={wingKIAS}
            onChange={(e) => setWingKIAS(Number(e.target.value))}
          />

          <div style={textStyle}>
            Initial Range on Bearing: {spawnNm.toFixed(2)} nm
          </div>
          <input
            style={sliderStyle}
            type="range"
            min="0.25"
            max="5"
            step="0.25"
            value={spawnNm}
            onChange={(e) => setSpawnNm(Number(e.target.value))}
          />

          <div style={textStyle}>Bearing Line:</div>
          <button
            onClick={() => setBearingMode("F-18")}
            style={buttonStyle(bearingMode === "F-18")}
          >
            45° Bearing Line (F-18)
          </button>
          <button
            onClick={() => setBearingMode("T-45")}
            style={buttonStyle(bearingMode === "T-45")}
          >
            30° Bearing Line (T-45)
          </button>

          <div style={sectionHeaderStyle}>Dynamic Parameters</div>
          <div style={textStyle}>
            Time to Join:{" "}
            {estimatedJoinSeconds === null
              ? "Stabilizes"
              : `${estimatedJoinSeconds.toFixed(0)} sec`}
          </div>
          <div style={textStyle}>
            ΔIAS or Airspeed Advantage: {wingKIAS - leadKIAS} kt
          </div>
          <div style={textStyle}>Airspeed (KTAS): {Math.round(wingKTAS)}</div>
        </div>
      </div>

      {/* SIM CONTROLS CENTERED ABOVE GRAPHIC */}
      <div style={simControlStyle}>
        <div style={textStyle}>Time Scale: {timeScale.toFixed(1)}x</div>
        <input
          style={{ ...sliderStyle, maxWidth: 440 }}
          type="range"
          min="0.1"
          max="10"
          step="0.1"
          value={timeScale}
          onChange={(e) => setTimeScale(Number(e.target.value))}
        />

        <div>
          <button onClick={resetJoin} style={simButtonStyle}>
            Reset Join
          </button>

          <button onClick={() => setPaused((p) => !p)} style={simButtonStyle}>
            {paused ? "Resume" : "Pause"}
          </button>
        </div>
      </div>

      <div style={graphicWrapStyle}>
        <svg width="660" height="500">
          {/* Orbit */}
          <circle
            cx={centerX}
            cy={centerY}
            r={orbitRadiusPx}
            fill="none"
            stroke="#3b82f6"
            strokeDasharray="8 6"
            strokeWidth={lineWeight}
          />

          {/* Post */}
          <circle cx={centerX} cy={centerY} r="5" fill="red" />

          {/* Radius */}
          <line
            x1={centerX}
            y1={centerY}
            x2={leadX}
            y2={leadY}
            stroke="rgba(248, 113, 113, 0.35)"
            strokeWidth={lineWeight}
            strokeDasharray="6 6"
          />

          <text
            x={radiusLabelX}
            y={radiusLabelY}
            fill="rgba(252, 165, 165, 0.75)"
            fontSize="13"
            fontWeight="bold"
          >
            Radius {turnRadiusNm.toFixed(2)} nm
          </text>

          {/* Lead heading line */}
          <line
            x1={leadX}
            y1={leadY}
            x2={leadHeadingX}
            y2={leadHeadingY}
            stroke="#38bdf8"
            strokeWidth={lineWeight}
            strokeDasharray="5 5"
          />

          {/* Bearing line */}
          <line
            x1={bearingLineStartX}
            y1={bearingLineStartY}
            x2={bearingLineEndX}
            y2={bearingLineEndY}
            stroke="yellow"
            strokeWidth={lineWeight}
            strokeDasharray="8 8"
          />

          <text
            x={bearingLabelX}
            y={bearingLabelY}
            fill="yellow"
            fontSize="12"
            fontWeight="bold"
            textAnchor="middle"
            transform={`rotate(${bearingHeading}, ${bearingLabelX}, ${bearingLabelY})`}
          >
            {bearingLineDeg}° Bearing Line
          </text>

          {/* Range line */}
          <line
            x1={wingX}
            y1={wingY}
            x2={leadX}
            y2={leadY}
            stroke="yellow"
            strokeWidth={lineWeight}
            strokeDasharray="6 4"
          />

          {/* Wingman nose / fuselage heading */}
          <line
            x1={wingX}
            y1={wingY}
            x2={wingNoseX}
            y2={wingNoseY}
            stroke="lime"
            strokeWidth={lineWeight}
            strokeDasharray="7 5"
          />

          {/* ATA arc */}
          <path
            d={ataArcPath}
            fill="none"
            stroke="lime"
            strokeWidth={lineWeight}
          />

          <text
            x={ataDegreesLabelX}
            y={ataDegreesLabelY}
            fill="lime"
            fontSize="13"
            fontWeight="bold"
          >
            ATA: {currentGeo.antennaTrainAngle.toFixed(1)}°
          </text>

          <text
            x={rangeLabelX}
            y={rangeLabelY}
            fill="yellow"
            fontSize="12"
            fontWeight="bold"
          >
            RNG: {rangeNm.toFixed(2)} nm
          </text>

          <text
            x={vcLabelX}
            y={vcLabelY}
            fill="yellow"
            fontSize="12"
            fontWeight="bold"
          >
            Vc: {currentGeo.closureKt.toFixed(1)} kt
          </text>

          {/* Lead aircraft */}
          <g transform={`translate(${leadX},${leadY})`}>
            <image
              href="${import.meta.env.BASE_URL}/super hornet_transparent background.png"
              x={-20}
              y={-20}
              width={40}
              height={40}
              transform={`rotate(${leadHeading + SPRITE_OFFSET})`}
            />
          </g>

          {/* Wingman aircraft */}
          <g transform={`translate(${wingX},${wingY})`}>
            <image
              href="/super hornet_transparent background.png"
              x={-20}
              y={-20}
              width={40}
              height={40}
              transform={`rotate(${wingHeading + SPRITE_OFFSET})`}
            />
          </g>
        </svg>
      </div>

      {/* TABLE BELOW GRAPHIC */}
      <table style={tableStyle}>
        <thead>
          <tr>
            <th style={cellStyle}>Range</th>
            <th style={cellStyle}>ATA</th>
            <th style={cellStyle}>Vc / Closure</th>
          </tr>
        </thead>

        <tbody>
          {tableRanges.map((r) => {
            const geo = solveGeometry(r);

            return (
              <tr key={r}>
                <td style={cellStyle}>{r.toFixed(2)} nm</td>
                <td style={cellStyle}>{geo.antennaTrainAngle.toFixed(1)}°</td>
                <td style={cellStyle}>{geo.closureKt.toFixed(1)} kt</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}