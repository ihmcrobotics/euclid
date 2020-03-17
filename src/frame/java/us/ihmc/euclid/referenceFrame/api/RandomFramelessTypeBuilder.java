package us.ihmc.euclid.referenceFrame.api;

import java.util.Random;

/**
 * Implement this interface to create builders for any type.
 * <p>
 * The objects created using this builder should contain random values changing from one object to
 * the next.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RandomFramelessTypeBuilder
{
   /**
    * Creates a new instance of the same object initialized with random values.
    *
    * @return the next object.
    */
   Object newInstance(Random random);
}