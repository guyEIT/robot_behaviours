import { describe, it, expect } from "vitest";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { parseBtXml, buildNameIndex } from "../bt-parser";
import { layoutBtGraph } from "../bt-layout";

const CAMPAIGN_XML = readFileSync(
  resolve(
    __dirname,
    "../../../../../robot_behaviors/trees/campaign/plate_imaging_campaign.xml",
  ),
  "utf-8",
);

const wrap = (inner: string, mainId = "T") => `
<root BTCPP_format="4" main_tree_to_execute="${mainId}">
  <BehaviorTree ID="${mainId}">${inner}</BehaviorTree>
</root>`;

describe("parseBtXml — control-flow header motifs", () => {
  it("classifies Sequence as control with controlKind=sequence", () => {
    const t = parseBtXml(
      wrap(`<Sequence name="s"><MoveTo name="a"/><MoveTo name="b"/></Sequence>`),
    );
    const seq = t.nodes.find((n) => n.btNodeType === "Sequence")!;
    expect(seq.category).toBe("control");
    expect(seq.controlKind).toBe("sequence");
  });

  it("attaches ordinals and Fallback dashed style on child edges", () => {
    const t = parseBtXml(
      wrap(`<Fallback name="f"><A name="a"/><B name="b"/><C name="c"/></Fallback>`),
    );
    const fb = t.nodes.find((n) => n.btNodeType === "Fallback")!;
    const childEdges = t.edges.filter((e) => e.source === fb.id);
    expect(childEdges).toHaveLength(3);
    expect(childEdges[0].ordinal).toBe(1);
    expect(childEdges[0].style).toBe("default");
    expect(childEdges[1].style).toBe("dashed");
    expect(childEdges[2].style).toBe("dashed");
  });

  it("labels IfThenElse children with cond/then/else", () => {
    const t = parseBtXml(
      wrap(`<IfThenElse><Cond name="c"/><Then name="t"/><Else name="e"/></IfThenElse>`),
    );
    const labels = t.edges
      .filter((e) => e.branchLabel !== undefined)
      .map((e) => e.branchLabel);
    expect(labels).toEqual(["cond", "then", "else"]);
  });

  it("labels WhileDoElse children with cond/do/else", () => {
    const t = parseBtXml(
      wrap(`<WhileDoElse><Cond name="c"/><Body name="b"/><Else name="e"/></WhileDoElse>`),
    );
    const labels = t.edges
      .filter((e) => e.branchLabel !== undefined)
      .map((e) => e.branchLabel);
    expect(labels).toEqual(["cond", "do", "else"]);
  });
});

describe("parseBtXml — container groups", () => {
  it("Repeat becomes a container with displayParams '× N'", () => {
    const t = parseBtXml(
      wrap(`<Repeat num_cycles="3"><MoveTo name="a"/></Repeat>`),
    );
    const repeat = t.nodes.find((n) => n.btNodeType === "Repeat")!;
    expect(repeat.containerKind).toBe("repeat");
    expect(repeat.category).toBe("container");
    expect(repeat.displayParams?.[0].label).toBe("× 3");
    const child = t.nodes.find((n) => n.btNodeType === "MoveTo")!;
    expect(child.parentId).toBe(repeat.id);
  });

  it("RetryUntilSuccessful becomes a container with '↻ N'", () => {
    const t = parseBtXml(
      wrap(`<RetryUntilSuccessful num_attempts="2"><A name="a"/></RetryUntilSuccessful>`),
    );
    const retry = t.nodes.find((n) => n.btNodeType === "RetryUntilSuccessful")!;
    expect(retry.containerKind).toBe("retry");
    expect(retry.displayParams?.[0].label).toBe("↻ 2");
  });

  it("KeepRunningUntilFailure becomes a container with '∞'", () => {
    const t = parseBtXml(
      wrap(`<KeepRunningUntilFailure><A name="a"/></KeepRunningUntilFailure>`),
    );
    const kuf = t.nodes.find((n) => n.btNodeType === "KeepRunningUntilFailure")!;
    expect(kuf.containerKind).toBe("keep_until_fail");
    expect(kuf.displayParams?.[0].label).toBe("∞");
  });

  it("SubTree becomes a container; referenced tree's children are nested under it", () => {
    const xml = `<root BTCPP_format="4" main_tree_to_execute="Outer">
      <BehaviorTree ID="Outer"><SubTree ID="Inner"/></BehaviorTree>
      <BehaviorTree ID="Inner"><Sequence name="s"><A name="a"/></Sequence></BehaviorTree>
    </root>`;
    const t = parseBtXml(xml);
    const sub = t.nodes.find((n) => n.btNodeType === "SubTree")!;
    expect(sub.containerKind).toBe("subtree");
    const seq = t.nodes.find((n) => n.btNodeType === "Sequence")!;
    expect(seq.parentId).toBe(sub.id);
  });
});

describe("parseBtXml — decorator chip collapse", () => {
  it("Inverter on a single child collapses to a chip on the child", () => {
    const t = parseBtXml(
      wrap(`<Inverter><MoveTo name="a"/></Inverter>`),
    );
    expect(t.nodes.find((n) => n.btNodeType === "Inverter")).toBeUndefined();
    const child = t.nodes.find((n) => n.btNodeType === "MoveTo")!;
    expect(child.decorators?.[0].kind).toBe("inverter");
    expect(child.decorators?.[0].label).toBe("!");
  });

  it("Timeout chip carries duration label", () => {
    const t = parseBtXml(
      wrap(`<Timeout msec="500"><A name="a"/></Timeout>`),
    );
    const child = t.nodes.find((n) => n.btNodeType === "A")!;
    expect(child.decorators?.[0].kind).toBe("timeout");
    expect(child.decorators?.[0].label).toBe("⏱ 500");
  });

  it("Multiple stacked decorators stack on the same child", () => {
    const t = parseBtXml(
      wrap(`<Inverter><Timeout msec="500"><A name="a"/></Timeout></Inverter>`),
    );
    const child = t.nodes.find((n) => n.btNodeType === "A")!;
    expect(child.decorators?.length).toBe(2);
    expect(child.decorators?.[0].kind).toBe("inverter");
    expect(child.decorators?.[1].kind).toBe("timeout");
  });
});

describe("parseBtXml — BlackboardCondition gate-on-edge vs node", () => {
  it("single-child BlackboardCondition collapses to an edge gate on its child", () => {
    const t = parseBtXml(
      wrap(`
        <Sequence name="s">
          <BlackboardCondition key="paused" expected="false">
            <Body name="body"/>
          </BlackboardCondition>
        </Sequence>
      `),
    );
    expect(
      t.nodes.find((n) => n.btNodeType === "BlackboardCondition"),
    ).toBeUndefined();
    const body = t.nodes.find((n) => n.btNodeType === "Body")!;
    const seq = t.nodes.find((n) => n.btNodeType === "Sequence")!;
    const edge = t.edges.find(
      (e) => e.source === seq.id && e.target === body.id,
    )!;
    expect(edge.gate).toEqual({ key: "paused", expected: "false" });
  });

  it("zero-child BlackboardCondition stays a leaf node", () => {
    const t = parseBtXml(
      wrap(`<Sequence name="s">
        <BlackboardCondition key="paused" expected="false"/>
        <A name="a"/>
      </Sequence>`),
    );
    const bb = t.nodes.find((n) => n.btNodeType === "BlackboardCondition");
    expect(bb).toBeDefined();
    expect(bb?.category).toBe("condition");
  });
});

describe("parseBtXml — campaign tree end-to-end", () => {
  it("parses the real campaign XML without throwing", () => {
    expect(() => parseBtXml(CAMPAIGN_XML)).not.toThrow();
  });

  it("identifies the outer KeepRunningUntilFailure as a container", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    const kuf = t.nodes.find((n) => n.btNodeType === "KeepRunningUntilFailure")!;
    expect(kuf.containerKind).toBe("keep_until_fail");
  });

  it("collapses the BlackboardCondition gate onto the edge into the inner Sequence", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    expect(
      t.nodes.find((n) => n.btNodeType === "BlackboardCondition"),
    ).toBeUndefined();
    const gates = t.edges.filter((e) => e.gate !== undefined);
    expect(gates.length).toBeGreaterThanOrEqual(1);
    const gate = gates[0].gate!;
    expect(gate.key).toBe("persistent.paused");
    expect(gate.expected).toBe("false");
  });

  it("frames the SubTree(PlateImageCycle) as a container and nests its children", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    const sub = t.nodes.find((n) => n.btNodeType === "SubTree")!;
    expect(sub.containerKind).toBe("subtree");
    const retry = t.nodes.find((n) => n.btNodeType === "RetryUntilSuccessful")!;
    expect(retry.parentId).toBe(sub.id);
    expect(retry.displayParams?.[0].label).toBe("↻ 2");
  });

  it("classifies LiconicFetch / ImagePlate as skill leaves and LogEvent as log leaf", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    const fetch = t.nodes.find((n) => n.btNodeType === "LiconicFetch")!;
    expect(fetch.leafCategory).toBe("skill");
    const log = t.nodes.find((n) => n.btNodeType === "LogEvent")!;
    expect(log.leafCategory).toBe("log");
    const wait = t.nodes.find((n) => n.btNodeType === "WaitUntil")!;
    expect(wait.leafCategory).toBe("timing");
  });

  it("buildNameIndex maps node names back to ids for the campaign tree", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    const idx = buildNameIndex(t.nodes);
    expect(idx.has("campaign_loop")).toBe(true);
    expect(idx.has("one_cycle")).toBe(true);
    expect(idx.has("cycle_retry")).toBe(true);
  });

  it("layoutBtGraph produces sized container groups with children parented", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    const layout = layoutBtGraph(t.nodes, t.edges);
    const containers = layout.nodes.filter(
      (n) => (n.data as { containerKind?: string }).containerKind !== undefined,
    );
    expect(containers.length).toBeGreaterThan(0);
    for (const c of containers) {
      expect(c.width ?? 0).toBeGreaterThan(0);
      expect(c.height ?? 0).toBeGreaterThan(0);
      expect(c.type).toBe("btContainer");
      const directChildren = layout.nodes.filter((n) => n.parentId === c.id);
      expect(directChildren.length).toBeGreaterThan(0);
    }
  });

  it("layoutBtGraph hides descendants when their container is collapsed", () => {
    const t = parseBtXml(CAMPAIGN_XML);
    const outerLoop = t.nodes.find(
      (n) => n.btNodeType === "KeepRunningUntilFailure",
    )!;
    const layout = layoutBtGraph(t.nodes, t.edges, new Set([outerLoop.id]));
    const outerInLayout = layout.nodes.find((n) => n.id === outerLoop.id)!;
    expect(outerInLayout.hidden).toBeFalsy();
    // Every descendant must be hidden.
    const descendants = layout.nodes.filter(
      (n) => n.id !== outerLoop.id && n.parentId !== undefined,
    );
    for (const d of descendants) {
      expect(d.hidden).toBe(true);
    }
    // Edges between hidden nodes are also hidden.
    const hiddenEdges = layout.edges.filter((e) => e.hidden);
    expect(hiddenEdges.length).toBeGreaterThan(0);
  });
});
