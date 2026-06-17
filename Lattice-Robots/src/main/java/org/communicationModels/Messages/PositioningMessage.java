package org.communicationModels.Messages;

import org.graphs.LatticeEdge;

public class PositioningMessage extends AbstractMessage {
    /**
     * The lattice edge that this positioning messages assigns to the recipient
     */
    private LatticeEdge currentEdge;
    /**
     * The ID of the root whose cycle is being built
     */
    private int rootId;
    /**
     * Whether the message is being used to assign or propogate a confirmation chain
     */
    private boolean isConfirmation;

    /**
     * Constructor for a positioning message
     * @param senderId the ID of the sender
     * @param recipient the ID of the recipient
     * @param currentEdge the edge being assigned to the recipient
     * @param rootId the ID of the root whose cycle is being built
     * @param isConfirmation whether the message is being used to assign or propagate a confirmation chain
     */
    public PositioningMessage(int senderId, int recipient, LatticeEdge currentEdge, int rootId, boolean isConfirmation) {
        super(senderId, recipient);
        this.currentEdge = currentEdge;
        this.rootId = rootId;
        this.isConfirmation = isConfirmation;
    }

    /**{@inheritDoc} */
    public String getMessageType() {
        return isConfirmation ? "Confirmation" : "Assignment";
    }

    /**
     * Determines whether the message is used for confirmation or assignment
     * @return true if the message is used for confirmation, false otherwise
    */
    public boolean isConfirmation() {
        return isConfirmation;
    }

    /**
     * Retrieves the root ID for the cycle
     * @return the root ID
     */
    public int getRootId() {
        return rootId;
    }

    /**
     * Retrieves the lattice edge assigned to the recipient
     * @return the lattice edge assigned
     */
    public LatticeEdge getCurrentEdge() {
        return currentEdge;
    }

    /**
     * Provides details of the message
     * @return a string with message details
     */
    private String getMessageInfo() {
        return super.messageInfo() + "\n"
        + "Root ID: " + this.rootId + "\n" 
        + "Lattice Edge: " + this.currentEdge;
    }

    @Override
    public String toString() {
        return getMessageInfo();
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }
        if(!super.equals(o)) {
            return false;
        }
        if(!(o instanceof PositioningMessage)) {
            return false;
        }
        PositioningMessage other = (PositioningMessage) o;

        return this.getRootId() == other.getRootId()
            && this.getCurrentEdge().equals(other.getCurrentEdge());
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), currentEdge, rootId);
    }
}
