package org.communicationModels.cycleBuildingComms.Messages;

import java.util.ArrayList;

import org.graphs.util.RigidBodyTransformation;

public class ChainMemberList {
    private ArrayList<Integer> chainList;
    private int hops;
    private RigidBodyTransformation measuredVoltage;

    public ChainMemberList() {
        chainList = new ArrayList<>();
        this.hops = 0;
        this.measuredVoltage = RigidBodyTransformation.identity();
    }

    public ChainMemberList(int robotId) {
        this.chainList = new ArrayList<>();
        chainList.add(robotId);
    }

    public ChainMemberList(ChainMemberList list, int robotId) {
        this.chainList = new ArrayList<>(list.chainList);
        this.chainList.add(robotId);
    }

    public int getHops() {
        return hops;
    }

    public RigidBodyTransformation getMeasuredVoltage() {
        return measuredVoltage;
    }

    public int getRootID() {
        return chainList.get(0);
    }

    public int getSenderID() {
        return chainList.getLast();
    }

    public ArrayList<Integer> getIDList() {
        return new ArrayList<>(chainList);
    }

    public boolean isInList(int ID) {
        return chainList.contains(ID);
    }

    public boolean isEmpty() {
        return chainList.isEmpty();
    }

    public int size() {
        return chainList.size();
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        } else if(!(o instanceof ChainMemberList)) {
            return false;
        }

        ChainMemberList other = (ChainMemberList) o;
        return this.chainList.equals(other.chainList);
    }
}
