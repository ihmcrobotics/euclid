package us.ihmc.euclid.geometry;

/**
 * Enumeration to help referencing between a minimum and a maximum bound.
 *
 * @author Sylvain Bertrand
 */
public enum Bound
{
   /**
    * Refers to lower values, i.e. towards -&infin;.
    */
   MIN
   {
      @Override
      public boolean isFirstBetter(double first, double second)
      {
         return first < second;
      }
   },
   /**
    * Refers to higher values, i.e. towards +&infin;.
    */
   MAX
   {
      @Override
      public boolean isFirstBetter(double first, double second)
      {
         return first > second;
      }
   };

   /**
    * Static final field holding the return from {@link #values()}. This field should be used in place
    * of calling values() for garbage-free operations.
    */
   public static final Bound[] values = values();

   /**
    * Compares the two values and indicates whether the first is farther in the direction of the bound.
    * <ul>
    * <li>For {@link #MIN}, this returns {@code true} if {@code first < second}.
    * <li>For {@link #MAX}, this returns {@code true} if {@code first > second}.
    * </ul>
    * 
    * @param first  the first value.
    * @param second the second value.
    * @return {@code true} if the first value is farther than the second according the bound.
    */
   public abstract boolean isFirstBetter(double first, double second);
}