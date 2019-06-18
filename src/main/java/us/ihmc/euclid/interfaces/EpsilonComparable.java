package us.ihmc.euclid.interfaces;

/**
 * This interface is used for any object that can be compared with a tolerance {@code epsilon}.
 *
 * @author Sylvain Bertrand
 * @param <T> the final type of the implementation of this interface.
 */
public interface EpsilonComparable<T>
{
   /**
    * Tests if {@code this} is equal to {@code other} to an {@code epsilon}. The test is usually
    * achieved on a per component basis. Sometimes a failing test does not necessarily mean that the
    * two objects are different in a geometric way.
    *
    * @param other   the other object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two objects are equal component-wise, {@code false} otherwise.
    */
   boolean epsilonEquals(T other, double epsilon);
}
