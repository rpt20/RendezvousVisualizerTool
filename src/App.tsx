import { useEffect, useMemo, useRef, useState, type CSSProperties } from "react";
import "./App.css";
import superHornet from "./assets/super-hornet.png";

type BearingMode = "F-18" | "T-45";

export default function App() {
  const centerX = 330;
  const centerY = 300;

  const SPRITE_OFFSET = 90;
  const leadKIAS = 250;
  const bankAngle = 30;
  const g = 32.174;
  const JOIN_THRESHOLD_NM = 0.01;

  const [bearingMode, setBearingMode] = useState<BearingMode>("F-18");
  const [leadAltitude, setLeadAltitude] = useState(10000);
  const [wingKIAS, setWingKIAS] = useState(250);
  const [spawnNm, setSpawnNm] = useState(1);
  const [timeScale, setTimeScale] = useState(2.5);
  const [paused, setPaused] = useState(false);

  const [orbitAngle, setOrbitAngle] = useState(0);
  const [rangeNm, setRangeNm] = useState(spawnNm);

  const lastFrameTimeRef = useRef<number | null>(null);

  const bearingLineDeg = bearingMode === "F-18" ? 45 : 30;

  const deg = (d: number) => (d * Math.PI) / 180;
  const radToDeg = (r: number) => (r * 180) / Math.PI;

  function normalizeAngle(a: number) {
    let x = a;
    while (x > 180) x -= 360;
    while (x < -180) x += 360;
    return x;
  }

  function clamp(value: number, min: number, max: number) {
    return Math.max(min, Math.min(max, value));
  }

  function headingFromVector(x: number, y: number) {
    return radToDeg(Math.atan2(y, x));
  }

  function kiasToKtas(kias: number, altitudeFt: number) {
    return kias * (1 + (altitudeFt / 1000) * 0.02);
  }

  function turnRadiusNmFromKtas(ktas: number) {
    const speedFps = ktas * 1.68781;

    const radiusFt =
      (speedFps * speedFps) / (g * Math.tan(deg(bankAngle)));

    return radiusFt / 6076.12;
  }

  function bankFromTurnRate(turnRateDegPerSecondInput: number, ktas: number) {
    const turnRateRadPerSecondInput = deg(turnRateDegPerSecondInput);
    const speedFps = ktas * 1.68781;

    return radToDeg(
      Math.atan((speedFps * turnRateRadPerSecondInput) / g)
    );
  }

  // =========================
  // AIRSPEED / TURN PHYSICS
  // =========================

  const leadKTAS = kiasToKtas(leadKIAS, leadAltitude);
  const wingKTAS = kiasToKtas(wingKIAS, leadAltitude);

  const trueAirspeedAdvantage = wingKTAS - leadKTAS;

  const turnRadiusNm = turnRadiusNmFromKtas(leadKTAS);

  const turnRateRadPerHour = leadKTAS / turnRadiusNm;
  const turnRateRadPerSecond = turnRateRadPerHour / 3600;
  const turnRateDegPerSecond = radToDeg(turnRateRadPerSecond);

  const orbitPeriodSeconds = 360 / turnRateDegPerSecond;

  // Smooth display scaling from 0 ft to 40,000 ft.
  // The real turn radius drives the math; this only scales the graphic.
  const minDisplayRadiusPx = 120;
  const maxDisplayRadiusPx = 215;

  const radiusAt0FtNm = turnRadiusNmFromKtas(kiasToKtas(leadKIAS, 0));
  const radiusAt40000FtNm = turnRadiusNmFromKtas(
    kiasToKtas(leadKIAS, 40000)
  );

  const radiusDisplayRatio = clamp(
    (turnRadiusNm - radiusAt0FtNm) /
      (radiusAt40000FtNm - radiusAt0FtNm),
    0,
    1
  );

  const orbitRadiusPx =
    minDisplayRadiusPx +
    (maxDisplayRadiusPx - minDisplayRadiusPx) * radiusDisplayRatio;

  const pixelsPerNm = orbitRadiusPx / turnRadiusNm;

  // Dynamic graphic height:
  // Bottom of the box tracks the circle bottom plus about 1/2 radius.
  // This keeps the playback slider tight to the circle without covering the graphic.
  const graphicViewBoxWidth = 660;
  const graphicViewBoxHeight = clamp(
    centerY + orbitRadiusPx * 1.5 + 18,
    490,
    650
  );

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
    function solveCore(inputRange: number) {
      const safeRange = Math.max(inputRange, 0.0001);

      const xRel = -safeRange * Math.sin(deg(bearingLineDeg));
      const yRel = safeRange * Math.cos(deg(bearingLineDeg));

      const wingPostX = xRel;
      const wingPostY = yRel - turnRadiusNm;

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

      if (inputRange <= 0.0001) {
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

      return {
        safeRange,
        xRel,
        yRel,
        vradKt,
        closureKt,
        canClose,
        wingHeadingMath,
        misalignment,
      };
    }

    const safeRange = Math.max(range, 0.0001);

    const rangePx = safeRange * pixelsPerNm;
    const bearingRad = deg(bearingHeading);

    const wingX = leadX + Math.cos(bearingRad) * rangePx;
    const wingY = leadY + Math.sin(bearingRad) * rangePx;

    const core = solveCore(range);

    const wingHeadingScreen = leadHeading - core.misalignment;

    // Signed ATA:
    // Negative = nose to the right side of bearing / LOS line.
    // Positive = nose to the left side of bearing / LOS line.
    const antennaTrainAngle = normalizeAngle(
      lineOfSightHeading - wingHeadingScreen
    );

    // Wing AOB estimate:
    // While driving up the bearing line, estimate the instantaneous heading
    // rate required by comparing this geometry to a small forward step.
    //
    // At the end of the join the wing is colocated with lead and should match
    // lead's turn, so AOB should be the lead AOB, not zero.
    const bankDt = 0.25;
    let wingBankDeg = bankAngle;

    if (range > JOIN_THRESHOLD_NM && core.canClose && core.closureKt > 0) {
      const nextRange = Math.max(
        JOIN_THRESHOLD_NM,
        range - (core.closureKt / 3600) * bankDt
      );

      const nextCore = solveCore(nextRange);

      const currentScreenHeading = leadHeading - core.misalignment;
      const nextLeadHeading = leadHeading - turnRateDegPerSecond * bankDt;
      const nextScreenHeading = nextLeadHeading - nextCore.misalignment;

      const headingChangeDeg = normalizeAngle(
        nextScreenHeading - currentScreenHeading
      );

      const rawBank = bankFromTurnRate(
        Math.abs(headingChangeDeg / bankDt),
        wingKTAS
      );

      wingBankDeg = clamp(rawBank, 0, 80);
    }

    if (range <= JOIN_THRESHOLD_NM) {
      wingBankDeg = bankAngle;
    }

    return {
      wingX,
      wingY,
      vradKt: core.vradKt,
      closureKt: core.closureKt,
      canClose: core.canClose,
      wingHeadingScreen,
      misalignment: core.misalignment,
      antennaTrainAngle,
      wingBankDeg,
    };
  }

  function estimateSimSecondsToJoin() {
    let r = spawnNm;
    let t = 0;
    const dt = 0.1;

    for (let i = 0; i < 120000; i++) {
      const geo = solveGeometry(r);

      if (!geo.canClose || geo.closureKt <= 0) {
        return null;
      }

      r -= (geo.closureKt / 3600) * dt;
      t += dt;

      if (r <= JOIN_THRESHOLD_NM) {
        return t;
      }
    }

    return null;
  }

  function resetJoin() {
    setRangeNm(spawnNm);
    setPaused(false);
    lastFrameTimeRef.current = null;
  }

  // =========================
  // SIM LOOP — WALL-CLOCK ACCURATE
  // =========================

  useEffect(() => {
    let animationFrameId = 0;

    function step(now: number) {
      if (lastFrameTimeRef.current === null) {
        lastFrameTimeRef.current = now;
      }

      const wallDtRaw = (now - lastFrameTimeRef.current) / 1000;
      lastFrameTimeRef.current = now;

      // Cap big resume/background spikes so the sim does not jump.
      const wallDt = clamp(wallDtRaw, 0, 0.1);
      const simDt = paused ? 0 : wallDt * timeScale;

      if (simDt > 0) {
        setOrbitAngle((a) => {
          return (a - turnRateDegPerSecond * simDt) % 360;
        });

        setRangeNm((r) => {
          if (r <= 0) return 0;

          const geo = solveGeometry(r);

          if (!geo.canClose || geo.closureKt <= 0) {
            return r;
          }

          return Math.max(
            0,
            r - (geo.closureKt / 3600) * simDt
          );
        });
      }

      animationFrameId = requestAnimationFrame(step);
    }

    animationFrameId = requestAnimationFrame(step);

    return () => cancelAnimationFrame(animationFrameId);
  }, [
    paused,
    timeScale,
    turnRateDegPerSecond,
    wingKIAS,
    leadAltitude,
    orbitAngle,
    leadHeading,
    bearingHeading,
    lineOfSightHeading,
    pixelsPerNm,
    bearingLineDeg,
  ]);

  useEffect(() => {
    lastFrameTimeRef.current = null;
  }, [paused, timeScale]);

  const currentGeo = solveGeometry(rangeNm);

  const estimatedSimJoinSeconds = useMemo(
    () => estimateSimSecondsToJoin(),
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

    while (r > 0 && values.length < 20) {
      values.push(Number(r.toFixed(2)));
      r -= 0.25;
    }

    if (values.length < 20) {
      values.push(0);
    }

    return values;
  }, [spawnNm]);

  const compactTableRanges = tableRanges.slice(0, 20);

  const wingX = currentGeo.wingX;
  const wingY = currentGeo.wingY;
  const wingHeading = currentGeo.wingHeadingScreen;

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

  const bankLabelX = rangeLabelX;
  const bankLabelY = rangeLabelY + 36;

  // =========================
  // STYLES
  // =========================

  const pageStyle: CSSProperties = {
    minHeight: "100vh",
    background: "#111827",
    color: "white",
    fontFamily: "Arial, sans-serif",
    padding: "14px 10px 36px",
    boxSizing: "border-box",
    overflowX: "hidden",
  };

  const titleStyle: CSSProperties = {
    textAlign: "center",
    fontSize: "clamp(30px, 9vw, 46px)",
    fontWeight: 800,
    color: "white",
    margin: "0 0 14px 0",
    lineHeight: 1,
    whiteSpace: "nowrap",
  };

  const centerColumnStyle: CSSProperties = {
    width: "min(900px, 100%)",
    margin: "0 auto",
  };

  const sliderLabelStyle: CSSProperties = {
    fontSize: 16,
    fontWeight: 700,
    lineHeight: 1.15,
    textAlign: "center",
    whiteSpace: "nowrap",
  };

  const sliderValueStyle: CSSProperties = {
    fontSize: 17,
    fontWeight: 800,
    lineHeight: 1.15,
    textAlign: "center",
    margin: "2px 0 5px",
    whiteSpace: "nowrap",
  };

  const sliderStyle: CSSProperties = {
    width: "100%",
    margin: 0,
    display: "block",
  };

  const buttonStyle = (active: boolean): CSSProperties => ({
    padding: "7px 12px",
    borderRadius: 8,
    border: active ? "2px solid #22c55e" : "1px solid #64748b",
    background: active ? "#163a23" : "#1f2937",
    color: "white",
    cursor: "pointer",
    fontSize: 14,
    margin: "4px 6px 0",
  });

  const graphicWrapStyle: CSSProperties = {
    width: "100%",
    maxWidth: "900px",
    aspectRatio: `${graphicViewBoxWidth} / ${graphicViewBoxHeight}`,
    margin: "0 auto",
    background: "#101827",
    boxShadow: "8px 8px 18px rgba(0,0,0,0.35)",
    borderRadius: 4,
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
    overflow: "hidden",
    position: "relative",
  };

  const playbackControlStyle: CSSProperties = {
    width: "min(420px, 88%)",
    margin: "4px auto 0",
    padding: "0 10px",
    textAlign: "center",
  };

  const timeScaleLabelStyle: CSSProperties = {
    fontSize: 13,
    fontWeight: 800,
    textAlign: "center",
    marginBottom: 2,
    color: "rgba(255,255,255,0.82)",
    textShadow: "0 1px 4px black",
    background: "transparent",
  };

  const smallSliderStyle: CSSProperties = {
    width: "100%",
    margin: "2px auto 4px",
    display: "block",
    background: "transparent",
  };

  const simButtonStyle: CSSProperties = {
    padding: "5px 11px",
    borderRadius: 7,
    border: "1px solid rgba(255,255,255,0.65)",
    background: "rgba(31,41,55,0.65)",
    color: "white",
    cursor: "pointer",
    fontSize: 12,
    margin: "2px 5px 0",
  };

  const paramBlockStyle: CSSProperties = {
    width: "min(500px, 100%)",
    margin: "24px auto 0",
    textAlign: "left",
  };

  const sectionHeaderStyle: CSSProperties = {
    fontSize: 24,
    textDecoration: "underline",
    margin: "18px 0 8px 0",
    lineHeight: 1.25,
    textAlign: "left",
  };

  const textStyle: CSSProperties = {
    fontSize: 22,
    lineHeight: 1.6,
    textAlign: "left",
  };

  const graphicInfoStyle: CSSProperties = {
    fill: "white",
    fontSize: 12,
    fontWeight: 700,
  };

  const compactHeaderStyle: CSSProperties = {
    fill: "white",
    fontSize: 10,
    fontWeight: 700,
  };

  const compactCellStyle: CSSProperties = {
    fill: "white",
    fontSize: 10,
    fontWeight: 500,
  };

  const compactStroke = "rgba(255,255,255,0.55)";

  // Compact table geometry
  const compactRowHeight = 14;
  const compactHeaderHeight = 20;
  const compactTableWidth = 192;
  const compactTableHeight =
    compactHeaderHeight + compactTableRanges.length * compactRowHeight;

  const compactCol1X = 5;
  const compactCol2X = 59;
  const compactCol3X = 96;
  const compactCol4X = 145;

  const compactDivider1X = 54;
  const compactDivider2X = 91;
  const compactDivider3X = 140;

  // =========================
  // RENDER
  // =========================

  return (
    <div style={pageStyle}>
      <style>
        {`
          .rv-bearing-row {
            text-align: center;
            margin: 0 auto 8px;
          }

          .rv-slider-row {
            display: grid;
            grid-template-columns: repeat(3, minmax(0, 1fr));
            gap: 14px;
            align-items: end;
            width: 100%;
            margin: 0 auto 8px;
          }

          .rv-slider-item {
            min-width: 0;
          }

          .rv-playback-controls {
            margin-top: 4px;
          }

          @media (max-width: 640px) {
            .rv-bearing-row {
              margin: 0 auto 4px;
            }

            .rv-slider-row {
              display: grid;
              grid-template-columns: 1fr;
              gap: 6px;
              margin: 0 auto 6px;
              width: min(360px, 100%);
            }

            .rv-slider-item {
              width: 100%;
            }

            .rv-playback-controls {
              margin-top: 4px;
            }
          }
        `}
      </style>

      <h1 style={titleStyle}>Rendezvous Visualizer</h1>

      <main style={centerColumnStyle}>
        {/* BEARING TOGGLE ABOVE SLIDERS */}
        <div className="rv-bearing-row">
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
        </div>

        {/* TOP SLIDERS — RESPONSIVE */}
        <div className="rv-slider-row">
          <div className="rv-slider-item">
            <div style={sliderLabelStyle}>Altitude</div>
            <div style={sliderValueStyle}>
              {leadAltitude.toLocaleString()} ft
            </div>
            <input
              style={sliderStyle}
              type="range"
              min="0"
              max="40000"
              step="500"
              value={leadAltitude}
              onChange={(e) => setLeadAltitude(Number(e.target.value))}
            />
          </div>

          <div className="rv-slider-item">
            <div style={sliderLabelStyle}>Wing KIAS</div>
            <div style={sliderValueStyle}>{wingKIAS}</div>
            <input
              style={sliderStyle}
              type="range"
              min="180"
              max="320"
              value={wingKIAS}
              onChange={(e) => setWingKIAS(Number(e.target.value))}
            />
          </div>

          <div className="rv-slider-item">
            <div style={sliderLabelStyle}>Start Range</div>
            <div style={sliderValueStyle}>{spawnNm.toFixed(2)} nm</div>
            <input
              style={sliderStyle}
              type="range"
              min="0.25"
              max="5"
              step="0.25"
              value={spawnNm}
              onChange={(e) => setSpawnNm(Number(e.target.value))}
            />
          </div>
        </div>

        {/* DYNAMIC GRAPHIC BOX */}
        <div style={graphicWrapStyle}>
          <svg
            width="100%"
            height="100%"
            viewBox={`0 0 ${graphicViewBoxWidth} ${graphicViewBoxHeight}`}
            preserveAspectRatio="xMidYMin meet"
            style={{
              display: "block",
              maxWidth: "100%",
              height: "100%",
              background: "#101827",
            }}
          >
            {/* Top-left in-graphic timing text */}
            <text x={14} y={24} style={graphicInfoStyle}>
              Time to Join:{" "}
              {estimatedSimJoinSeconds === null
                ? "Stabilizes"
                : `${estimatedSimJoinSeconds.toFixed(0)} sec`}
            </text>
            <text x={14} y={42} style={graphicInfoStyle}>
              Time for 360° Turn: {orbitPeriodSeconds.toFixed(0)} sec
            </text>

            {/* Compact in-graphic table */}
            <g transform="translate(14, 58)">
              <rect
                x={0}
                y={0}
                width={compactTableWidth}
                height={compactTableHeight}
                fill="rgba(17,24,39,0.75)"
                stroke={compactStroke}
                strokeWidth={1}
              />

              {/* vertical column dividers */}
              <line
                x1={compactDivider1X}
                y1={0}
                x2={compactDivider1X}
                y2={compactTableHeight}
                stroke={compactStroke}
                strokeWidth={1}
              />
              <line
                x1={compactDivider2X}
                y1={0}
                x2={compactDivider2X}
                y2={compactTableHeight}
                stroke={compactStroke}
                strokeWidth={1}
              />
              <line
                x1={compactDivider3X}
                y1={0}
                x2={compactDivider3X}
                y2={compactTableHeight}
                stroke={compactStroke}
                strokeWidth={1}
              />

              {/* header divider */}
              <line
                x1={0}
                y1={compactHeaderHeight}
                x2={compactTableWidth}
                y2={compactHeaderHeight}
                stroke={compactStroke}
                strokeWidth={1}
              />

              <text x={compactCol1X} y={14} style={compactHeaderStyle}>
                Range
              </text>
              <text x={compactCol2X} y={14} style={compactHeaderStyle}>
                ATA
              </text>
              <text x={compactCol3X} y={14} style={compactHeaderStyle}>
                Vc
              </text>
              <text x={compactCol4X} y={14} style={compactHeaderStyle}>
                AOB
              </text>

              {compactTableRanges.map((r, i) => {
                const geo = solveGeometry(r);
                const y = compactHeaderHeight + 11 + i * compactRowHeight;

                return (
                  <g key={r}>
                    <text x={compactCol1X} y={y} style={compactCellStyle}>
                      {r.toFixed(2)} nm
                    </text>
                    <text x={compactCol2X} y={y} style={compactCellStyle}>
                      {geo.antennaTrainAngle.toFixed(1)}°
                    </text>
                    <text x={compactCol3X} y={y} style={compactCellStyle}>
                      {geo.closureKt.toFixed(0)} kts
                    </text>
                    <text x={compactCol4X} y={y} style={compactCellStyle}>
                      {geo.wingBankDeg.toFixed(0)}°
                    </text>
                  </g>
                );
              })}
            </g>

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

            <text
              x={bankLabelX}
              y={bankLabelY}
              fill="yellow"
              fontSize="12"
              fontWeight="bold"
            >
              AOB: {currentGeo.wingBankDeg.toFixed(0)}°
            </text>

            {/* Lead aircraft */}
            <g transform={`translate(${leadX},${leadY})`}>
              <image
                href={superHornet}
                x={-15}
                y={-8}
                width={30}
                height={30}
                transform={`rotate(${leadHeading + SPRITE_OFFSET})`}
              />
            </g>

            {/* Wingman aircraft */}
            <g transform={`translate(${wingX},${wingY})`}>
              <image
                href={superHornet}
                x={-15}
                y={-8}
                width={30}
                height={30}
                transform={`rotate(${wingHeading + SPRITE_OFFSET})`}
              />
            </g>
          </svg>
        </div>

        {/* PLAYBACK CONTROLS DIRECTLY BELOW DYNAMIC GRAPHIC */}
        <div className="rv-playback-controls" style={playbackControlStyle}>
          <div style={timeScaleLabelStyle}>{timeScale.toFixed(1)}x</div>

          <input
            style={smallSliderStyle}
            type="range"
            min="0.1"
            max="10"
            step="0.1"
            value={timeScale}
            onChange={(e) => setTimeScale(Number(e.target.value))}
          />

          <button onClick={resetJoin} style={simButtonStyle}>
            Restart
          </button>

          <button onClick={() => setPaused((p) => !p)} style={simButtonStyle}>
            {paused ? "Resume" : "Pause"}
          </button>
        </div>

        {/* FIXED PARAMETERS */}
        <div style={paramBlockStyle}>
          <div style={sectionHeaderStyle}>Fixed Parameters</div>
          <div style={textStyle}>Lead Airspeed (KIAS): {leadKIAS}</div>
          <div style={textStyle}>Angle of Bank: {bankAngle}°</div>
        </div>

        {/* DYNAMIC PARAMETERS */}
        <div style={paramBlockStyle}>
          <div style={sectionHeaderStyle}>Dynamic Parameters</div>
          <div style={textStyle}>Turn Radius: {turnRadiusNm.toFixed(2)} nm</div>
          <div style={textStyle}>
            True Airspeed Advantage: {trueAirspeedAdvantage.toFixed(1)} kt
          </div>
          <div style={textStyle}>
            Lead Airspeed (KTAS): {Math.round(leadKTAS)}
          </div>
          <div style={textStyle}>
            Wing Airspeed (KTAS): {Math.round(wingKTAS)}
          </div>
        </div>
      </main>
    </div>
  );
}