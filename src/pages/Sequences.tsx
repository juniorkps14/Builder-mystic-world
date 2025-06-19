import { SequenceManager } from "@/components/ros/SequenceManager";

const Sequences = () => {
  return (
    <div>
      <div className="mb-6">
        <h1 className="text-3xl font-bold">Sequence Management</h1>
        <p className="text-muted-foreground">
          Create and manage task sequences with sequential and parallel
          execution modes
        </p>
      </div>
      <SequenceManager />
    </div>
  );
};

export default Sequences;
