package us.ihmc.euclid.referenceFrame.api;

import static us.ihmc.euclid.referenceFrame.api.MethodSignature.getMethodSimpleName;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;

public class ReflectionBasedBuilders
{
   private final static Set<Class<?>> frameRandomGeneratorLibrary = new HashSet<>();
   private final static Set<Class<?>> framelessRandomGeneratorLibrary = new HashSet<>();

   private final static Map<Class<?>, RandomFrameTypeBuilder> frameTypeBuilders = new HashMap<>();
   private final static Map<Class<?>, RandomFramelessTypeBuilder> framelessTypeBuilders = new HashMap<>();

   static
   {
      framelessTypeBuilders.put(double.class, random -> EuclidCoreRandomTools.nextDouble(random, 10.0));
      framelessTypeBuilders.put(float.class, random -> (float) EuclidCoreRandomTools.nextDouble(random, 10.0));
      framelessTypeBuilders.put(boolean.class, random -> random.nextBoolean());
      framelessTypeBuilders.put(int.class, random -> random.nextInt(1000) - 500);
      framelessTypeBuilders.put(char.class, random -> (char) (random.nextInt(1000) - 500));
      framelessTypeBuilders.put(long.class, random -> (long) (random.nextInt(1000) - 500));

      framelessTypeBuilders.put(double[].class, random -> random.doubles(20, -10.0, 10.0).toArray());
      framelessTypeBuilders.put(float[].class, random -> nextFloatArray(random));
      framelessTypeBuilders.put(boolean[].class, random -> nextBooleanArray(random));
      framelessTypeBuilders.put(int[].class, random -> random.ints(20, -100, 100).toArray());
      framelessTypeBuilders.put(char[].class, random -> nextCharArray(random));
      framelessTypeBuilders.put(long[].class, random -> random.longs(20, -100, 100).toArray());

      framelessTypeBuilders.put(Transform.class, random -> EuclidCoreRandomTools.nextAffineTransform(random));
      framelessTypeBuilders.put(RigidBodyTransformReadOnly.class, random -> EuclidCoreRandomTools.nextRigidBodyTransform(random));
      framelessTypeBuilders.put(DenseMatrix64F.class, random -> RandomMatrices.createRandom(20, 20, random));
   }

   public static void registerFrameRandomGeneratorClasses(Class<?>... classes)
   {
      frameRandomGeneratorLibrary.addAll(Arrays.asList(classes));
   }

   public static void registerFramelessRandomGeneratorClasses(Class<?>... classes)
   {
      framelessRandomGeneratorLibrary.addAll(Arrays.asList(classes));
   }

   public static void registerFrameTypeBuilderSmart(Class<?> frameType)
   {
      registerFrameTypeBuilder(frameType, searchFrameGenerator(frameType));
   }

   public static void registerFrameTypeBuilder(Class<?> frameType, RandomFrameTypeBuilder frameTypeBuilder)
   {
      frameTypeBuilders.put(frameType, frameTypeBuilder);
   }

   public static void registerFramelessTypeBuilderSmart(Class<?> framelessType)
   {
      registerFramelessTypeBuilder(framelessType, searchFramelessGenerator(framelessType, true));
   }

   public static void registerFramelessTypeBuilder(Class<?> framelessType, RandomFramelessTypeBuilder framelessTypeBuilder)
   {
      framelessTypeBuilders.put(framelessType, framelessTypeBuilder);
   }

   static Object next(Random random, ReferenceFrame frame, Class<?> type)
   {
      Object object = newInstanceOfFrameType(random, type, frame);
      if (object != null)
         return object;
      object = newInstanceOfFramelessType(random, type);
      if (object != null)
         return object;
      return newInstanceOfGenericType(random, type);
   }

   static Object[] newInstance(Random random, ReferenceFrame frame, Class<?>[] types)
   {
      Object[] instances = new Object[types.length];

      for (int i = 0; i < types.length; i++)
      {
         Class<?> type = types[i];

         if (type.isArray() && !type.getComponentType().isPrimitive())
         {
            Class<?> componentType = type.getComponentType();

            Object[] array = (Object[]) Array.newInstance(componentType, random.nextInt(15));

            for (int j = 0; j < array.length; j++)
            {
               array[j] = next(random, frame, componentType);
               if (array[j] == null)
                  return null;
            }
            instances[i] = array;
         }
         else
         {
            instances[i] = next(random, frame, type);
            if (instances[i] == null)
               return null;
         }
      }
      return instances;
   }

   private static Object newInstanceOfFramelessType(Random random, Class<?> type)
   {
      RandomFramelessTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, RandomFramelessTypeBuilder> entry : ReflectionBasedBuilders.framelessTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(random);
   }

   private static Object newInstanceOfFrameType(Random random, Class<?> type, ReferenceFrame referenceFrame)
   {
      RandomFrameTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, RandomFrameTypeBuilder> entry : ReflectionBasedBuilders.frameTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(random, referenceFrame);
   }

   private static Object newInstanceOfGenericType(Random random, Class<?> type)
   {
      if (Collection.class.equals(type))
      {
         return null;
      }

      if (Object.class.equals(type))
      {
         return null;
      }

      RandomFramelessTypeBuilder generator = searchFramelessGenerator(type, false);

      if (generator != null)
      {
         framelessTypeBuilders.put(type, generator);
         return generator.newInstance(random);
      }

      throw new RuntimeException("Unknown class: " + type.getSimpleName());
      //      try
      //      {
      //         return type.getConstructor().newInstance();
      //      }
      //      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
      //            | SecurityException e)
      //      {
      //         throw new RuntimeException("Could not instantiate an object of the type: " + type.getSimpleName() + " " + type);
      //      }
   }

   private static RandomFramelessTypeBuilder searchFramelessGenerator(Class<?> type, boolean required)
   {
      List<Method> searchResult = new ArrayList<>();

      List<Method> allMethods = framelessRandomGeneratorLibrary.stream().flatMap(generatorClass -> Stream.of(generatorClass.getMethods()))
                                                               .collect(Collectors.toList());

      for (Method method : allMethods)
      {
         if (!Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
            continue;

         if (method.getParameterCount() != 1)
            continue;

         if (method.getParameterTypes()[0] != Random.class)
            continue;

         if (!type.isAssignableFrom(method.getReturnType()))
            continue;

         searchResult.add(method);
      }

      int nameLength = Integer.MAX_VALUE;
      Method randomGenerator = null;

      for (Method method : searchResult)
      {
         if (method.getName().length() < nameLength)
         {
            nameLength = method.getName().length();
            randomGenerator = method;
         }
      }

      System.out.println("Random generator for " + type.getSimpleName() + ", " + getMethodSimpleName(randomGenerator));

      if (!required && randomGenerator == null)
      {
         System.err.println("Unable to find a random generator for the type " + type.getSimpleName());
         return null;
      }

      Objects.requireNonNull(randomGenerator);
      final Method finalRandomGenerator = randomGenerator;

      return random ->
      {
         try
         {
            return finalRandomGenerator.invoke(null, random);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      };
   }

   private static RandomFrameTypeBuilder searchFrameGenerator(Class<?> mutableFrameMutableType)
   {
      List<Method> searchResult = new ArrayList<>();

      List<Method> allMethods = frameRandomGeneratorLibrary.stream().flatMap(generatorClass -> Stream.of(generatorClass.getMethods()))
                                                           .collect(Collectors.toList());

      for (Method method : allMethods)
      {
         if (!Modifier.isStatic(method.getModifiers()) || !Modifier.isPublic(method.getModifiers()))
            continue;

         if (method.getParameterCount() != 2)
            continue;

         if (method.getParameterTypes()[0] != Random.class || method.getParameterTypes()[1] != ReferenceFrame.class)
            continue;

         if (!mutableFrameMutableType.isAssignableFrom(method.getReturnType()))
            continue;

         searchResult.add(method);
      }

      int nameLength = Integer.MAX_VALUE;
      Method randomGenerator = null;

      for (Method method : searchResult)
      {
         if (method.getName().length() < nameLength)
         {
            nameLength = method.getName().length();
            randomGenerator = method;
         }
      }

      Objects.requireNonNull(randomGenerator);

      System.out.println("Random generator for " + mutableFrameMutableType.getSimpleName() + ", " + getMethodSimpleName(randomGenerator));

      final Method finalRandomGenerator = randomGenerator;

      return (random, frame) ->
      {
         try
         {
            return (ReferenceFrameHolder) finalRandomGenerator.invoke(null, random, frame);
         }
         catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      };
   }

   private static float[] nextFloatArray(Random random)
   {
      float[] next = new float[20];
      for (int i = 0; i < next.length; i++)
         next[i] = random.nextFloat();
      return next;
   }

   private static boolean[] nextBooleanArray(Random random)
   {
      boolean[] next = new boolean[20];
      for (int i = 0; i < next.length; i++)
         next[i] = random.nextBoolean();
      return next;
   }

   private static char[] nextCharArray(Random random)
   {
      char[] next = new char[20];
      for (int i = 0; i < next.length; i++)
         next[i] = (char) random.nextInt();
      return next;
   }
}
