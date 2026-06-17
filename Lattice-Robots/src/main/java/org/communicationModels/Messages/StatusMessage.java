package org.communicationModels.Messages;


public class StatusMessage extends AbstractMessage {
    private boolean isSuccessful;

    public StatusMessage(int senderId, int recipient, boolean isSuccessful) {
        super(senderId, recipient);
        this.isSuccessful = isSuccessful;
    }

    public boolean isSuccessful() {
        return isSuccessful;
    }
    /**{@inheritDoc}*/
    public String getMessageType() {
        return "Status";
    }

    @Override
    public String toString() {
        return super.toString() + "\n"
            + "Status: " + (isSuccessful ? "Success" : "Failure");
    
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

        return this.isSuccessful() == other.isSuccessful();
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(super.hashCode(), isSuccessful);
    }
}
