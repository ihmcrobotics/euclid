package us.ihmc.euclid.interfaces;

public interface EuclidGeometry
{
   /**
    * Tests if {@code this} is equal to {@code other} to an {@code epsilon}. The test is usually
    * achieved on a per component basis. Sometimes a failing test does not necessarily mean that the
    * two objects are different in a geometric way.
    *
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two objects are equal component-wise, {@code false} otherwise.
    */
   boolean epsilonEquals(Object object, double epsilon);
   
   /**
    * Tests if {@code this} and {@code other} represent the same geometry to an {@code epsilon}.
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will be
    * considered geometrically equal if they are at a distance from each other that is less or equal
    * than {@code epsilon}.
    * </p>
    *
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing the two objects, usually refers to a distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   boolean geometricallyEquals(Object object, double epsilon);
   
   /**
    * Provides a {@code String} representation of this EuclidGeometry.
    *
    * @param format the format to use for each number.
    * @return the {@code String} representing this tuple4D.
    */
   String toString(String format);
}