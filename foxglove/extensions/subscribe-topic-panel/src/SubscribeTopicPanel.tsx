import { PanelExtensionContext, RenderState, Topic, MessageEvent, Immutable } from "@foxglove/studio";
import { JsonViewer } from "@textea/json-viewer";
import { useLayoutEffect, useEffect, useState, useMemo } from "react";
import { createRoot } from "react-dom/client";

type State = {
  topic?: string;
  colorScheme?: RenderState["colorScheme"];
  topics?: readonly Topic[];
  message?: MessageEvent;
};

function SubscribeTopicPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Restore our state from the layout via the context.initialState property.
  const [state, setState] = useState<State>(() => {
    return context.initialState as State;
  });

  // Get topics
  const topics = useMemo(() => state.topics ?? [], [state.topics]);

  useEffect(() => {
    // Save our state to the layout when the topic changes.
    context.saveState({ topic: state.topic });

    if (state.topic) {
      // Subscribe to the new image topic when a new topic is chosen.
      context.subscribe([{ topic: state.topic }]);
    }
  }, [context, state.topic]);

  // Choose our first available image topic as a default once we have a list of topics available.
  useEffect(() => {
    if (state.topic == undefined) {
      setState((oldState) => ({ ...oldState, topic: topics[0]?.name }));
    }
  }, [state.topic, topics]);

  // Setup our onRender function and start watching topics and currentFrame for messages.
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setState((oldState) => ({
        ...oldState,
        topics: renderState.topics,
        colorScheme: renderState.colorScheme,
      }));

      // Save the most recent message on our topic.
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const lastFrame = renderState.currentFrame[renderState.currentFrame.length - 1] as MessageEvent;

        setState((oldState) => ({ ...oldState, message: lastFrame }));
      }
    };

    context.watch("topics");
    context.watch("currentFrame");
    context.watch("colorScheme");
  }, [context]);

  // Call our done function at the end of each render.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ height: "100%", padding: "1rem" }}>
      <h2>Subscribe Topic</h2>
      <div>
        <label>Choose a topic to display: </label>
        <select
          value={state.topic}
          onChange={(event) => {
            setState((oldState) => ({ ...oldState, topic: event.target.value }));
          }}
          style={{ flex: 1 }}
        >
          {topics.map((topic) => (
            <option key={topic.name} value={topic.name}>
              {topic.name}
            </option>
          ))}
        </select>

        <JsonViewer
          rootName={false}
          value={state.message as object}
          indentWidth={2}
          theme={state.colorScheme}
          enableClipboard={false}
          displayDataTypes={false}
          maxDisplayLength={10}
        />
      </div>
    </div>
  );
}

export function initSubscribeTopicPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SubscribeTopicPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}