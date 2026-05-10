import { Handle, Position } from "@xyflow/react";
import { memo } from "react";

function BlockNodeComponent({ data }) {
  const { id, kind, inputs, outputs } = data;
  return (
    <div className="block-node">
      <div className="block-title">
        {id} <small>({kind})</small>
      </div>
      <div className="block-ports">
        <div className="block-in">
          {inputs.map((p) => (
            <div className="port-row" key={`in-${p}`}>
              <Handle
                type="target"
                position={Position.Left}
                id={`in_${p}`}
                className="port-handle"
              />
              <span>{p}</span>
            </div>
          ))}
        </div>
        <div className="block-out">
          {outputs.map((p) => (
            <div className="port-row" key={`out-${p}`}>
              <span>{p}</span>
              <Handle
                type="source"
                position={Position.Right}
                id={`out_${p}`}
                className="port-handle"
              />
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

export const BlockNode = memo(BlockNodeComponent);
