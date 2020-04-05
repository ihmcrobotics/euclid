package us.ihmc.euclid.referenceFrame.api;

import java.util.Random;

/**
 * Implement this interface to create builders for any type.
 * <p>
 * The objects created using this builder should contain random values changing from one object to
 * the next.
 * </p>
 * <p>
 * This interface is part of the API testing framework.
 * </p>
 *
 * @see EuclidFrameAPITester
 * @author Sylvain Bertrand
 */
public interface RandomFramelessTypeBuilder
{
   /**
    * Creates a new instance of the same object initialized with random values.
    *
    * @param random the random generator to use for creating the next object.
    * @return the next object.
    */
   Object newInstance(Random random);
}