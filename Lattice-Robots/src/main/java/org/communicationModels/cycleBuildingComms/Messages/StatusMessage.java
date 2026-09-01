package org.communicationModels.cycleBuildingComms.Messages;

/**
 * The verdict on a walk, travelling back down the chain the walk came up.
 *
 * <p><strong>A status is born at the initiator and dies at the initiator.</strong> A certificate
 * laps its face and returns to the robot that minted it as a {@code PositioningMessage}; that
 * robot judges it and emits one of these to whoever handed it back. The status then walks the
 * chain <em>backwards</em> -- every robot forwards it to the parent of the tuple whose child sent
 * it -- until it arrives at the minter again, which is where {@link #getInitiatorId()} stops it.
 *
 * <p>That second lap is not ceremony. It is what lets every robot along the face record the
 * outcome against its own corner, which is what makes a duplicate certificate on an already-built
 * face a no-op rather than a second attempt.
 *
 * <p><strong>{@link #getInitiatorId()} is the termination condition and is never null.</strong>
 * Without it a status has no way to recognise the robot it is addressed to and circulates
 * forever. It is carried as a scalar of its own rather than read off {@link #getCertificate()}
 * because the certificate is optional here -- a failure raised by a robot that ran out of
 * candidates reports on a walk whose certificate it may not be able to hand back -- and a
 * termination condition that is sometimes absent is not one.
 */
public class StatusMessage extends AbstractMessage {
    private boolean isSuccessful;
    private int originVertexID;
    private int originOutgoingEdgeID;
    private int initiatorId;
    private VoltageCertificate certificate;

    /** A status carrying no certificate -- used where the walk never had one to return. */
    public StatusMessage(int senderId, int recipient, boolean isSuccessful, int originVertexID, int originOutgoingEdgeID, int initiatorId) {
        this(senderId, recipient, isSuccessful, originVertexID, originOutgoingEdgeID, initiatorId, null);
    }

    /**
     * @param initiatorId the robot that minted the walk this status reports on, and the only one
     *                    that may act on it. Every relayer passes it through untouched; the robot
     *                    whose own id it names marks its corner, drops its attempt tuple, and does
     *                    <em>not</em> forward.
     * @param certificate the certificate this status is reporting on, handed back to the
     *                    parent unchanged. Carrying it here is what lets a robot avoid
     *                    keeping a copy of its own: whoever made the offer gets the
     *                    certificate back and can re-offer with it directly.
     */
    public StatusMessage(int senderId, int recipient, boolean isSuccessful, int originVertexID, int originOutgoingEdgeID, int initiatorId, VoltageCertificate certificate) {
        super(senderId, recipient);
        this.isSuccessful = isSuccessful;
        this.originVertexID = originVertexID;
        this.originOutgoingEdgeID = originOutgoingEdgeID;
        this.initiatorId = initiatorId;
        this.certificate = certificate;
    }

    /** The certificate that travelled with the walk this status reports on. */
    public VoltageCertificate getCertificate() {
        return certificate;
    }

    /**
     * The robot that minted the walk this status reports on -- the one robot permitted to act on
     * it, and the point at which it stops travelling.
     */
    public int getInitiatorId() {
        return initiatorId;
    }

    public boolean isSuccessful() {
        return isSuccessful;
    }

    public int getOriginVertexID() {
        return originVertexID;
    }

    public int getOriginOutgoingEdgeID() {
        return originOutgoingEdgeID;
    }

    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Status";
    }

    public int getPriority() {
        return 1;
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Status: " + (isSuccessful ? "Success" : "Failure") + "\n"
            + "Initiator ID: " + initiatorId + "\n"
            + "Beginning Edge of Cycle Vertex ID: " + originVertexID + " \n"
            + "Beginning Edge of Cycle Edge ID: " + originOutgoingEdgeID;

    }

    @Override
    public boolean equals(Object o) {
        if(this == o) {
            return true;
        }
        if(!super.equals(o)) {
            return false;
        }
        if(!(o instanceof StatusMessage)) {
            return false;
        }

        StatusMessage other = (StatusMessage) o;

        return this.isSuccessful() == other.isSuccessful()
            && initiatorId == other.getInitiatorId()
            && originVertexID == other.getOriginVertexID()
            && originOutgoingEdgeID == other.getOriginOutgoingEdgeID();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), isSuccessful, initiatorId, originVertexID, originOutgoingEdgeID);
    }
}
