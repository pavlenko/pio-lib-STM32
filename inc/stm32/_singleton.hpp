#ifndef __SINGLETON__
#define __SINGLETON__

template <typename T>
class Singleton
{
public:
    Singleton(const Singleton&) = delete;
    const Singleton& operator = (const Singleton&) = delete;

    static constexpr T& instance() { return _instance; }

protected:
    Singleton() = default;

private:
    inline static constinit T _instance;
};

// Example derived class
class SingletonImpl : public Singleton<SingletonImpl>
{
public:
    friend class Singleton;

private:
    SingletonImpl() = default;
};

// Example access function
[[maybe_unused]] static SingletonImpl& SingletonF()
{
    return SingletonImpl::instance();
}

#endif // __SINGLETON__
