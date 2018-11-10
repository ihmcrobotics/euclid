package us.ihmc.euclid.interfaces;

/**
 * Base interface for any object that is that is settable with other objects of its own type.
 *
 * @author Sylvain Bertrand
 * @param <T> the final type of the implementation of this interface.
 */
public interface Settable<T>
{
   /**
    * Copies the values from {@code other} into this object.
    *
    * @param other the other object to copy the values from. Not modified.
    */
   void set(T other);
}
