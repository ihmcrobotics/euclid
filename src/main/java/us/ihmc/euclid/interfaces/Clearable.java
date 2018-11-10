package us.ihmc.euclid.interfaces;

/**
 * Interface used for geometry objects which data can be cleared by either resetting it with
 * {@link #setToZero()} or invalidating it with {@link #setToNaN()}.
 *
 * @author Sylvain Bertrand
 */
public interface Clearable
{

   /**
    * Tests if this object contains at least one value equal to {@link Double#NaN}.
    *
    * @return {@code true} if this object contains at least one value equal to {@link Double#NaN},
    *         {@code false} otherwise.
    */
   boolean containsNaN();

   /**
    * Invalidate this object by setting its values to {@link Double#NaN}.
    */
   void setToNaN();

   /**
    * Reset this object values.
    */
   void setToZero();

}