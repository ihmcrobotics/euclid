package us.ihmc.euclid.shape.primitives.interfaces;

/**
 * Listener that be used to detect changes in the object it was registered.
 * 
 * @author Sylvain Bertrand
 */
public interface Shape3DChangeListener
{
   /**
    * Notifies that the object has changed.
    */
   void changed();
}
