package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;

import org.graphs.voltage.VoltageGraph;

public class FaceObligation {
    private int parentId;
    private Integer childId;
    private int edgeId;
    private List<Integer> unableToDoAssignmentIds;

    public FaceObligation(int parentId, int edgeId) {
        setVariables(parentId, null, edgeId);
    }

    public void fulfil(int childId) {
        this.childId = childId;
    }

    public void release() {
        this.childId = null;
    }

    public boolean isUnfilled() {
        return this.childId == null;
    }

    public boolean matchesChild(int childId) {
        return this.childId != null && this.childId == childId;
    }

    public int faceId(VoltageGraph graph) {
        return graph.getHalfEdgeById(edgeId).getFace().getId();
    }

    public int getParentId() {
        return parentId;
    }

    public Integer getChildId() {
        return childId;
    }

    public int getEdgeId() {
        return edgeId;
    }

    public void ban(int bannedId) {
        this.unableToDoAssignmentIds.add(bannedId);
    }

    public boolean isBanned(int robotId) {
        return this.unableToDoAssignmentIds.contains(robotId);
    }

    private void setVariables(int parentId, Integer childId, int edgeId) {
        this.parentId = parentId;
        this.childId = childId;
        this.edgeId = edgeId;
        this.unableToDoAssignmentIds = new ArrayList<>();
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) return true;

        if(!(o instanceof FaceObligation)) return false;

        FaceObligation other = (FaceObligation) o;
        return this.getParentId() == other.getParentId()
            && this.getChildId() == other.getChildId()
            && this.getEdgeId() == other.getEdgeId();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(parentId, childId, edgeId);
    }
}
