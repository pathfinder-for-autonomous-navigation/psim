
import itertools
import re
import sys
import yaml

# Regular expression used to check the validity on configuration fields
_re_arg = re.compile(r'^[a-z][a-z_]*$')
_re_model = re.compile(r'^[A-Z][A-Za-z0-9]*$')
_re_variable = re.compile(r'^([a-zA-Z]|\{[a-z][a-z_]*\})([a-zA-Z0-9_\.]|\{[a-z][a-z_]*\})*$')
_re_type = re.compile(r'^([A-Z][a-z0-9])( |[A-Z][a-z0-9])*$')
_re_modifier = re.compile(r'[A-Z][a-z0-9]*')
_re_comment = re.compile(r'^[^"\n\t]+$')
_re_replace = re.compile(r'\{[a-z][a-z_]*\}')


class Argument(object):
    """Represents a model argument.
    """
    def __init__(self, name):
        super(Argument, self).__init__()

        self._name = str(name)
        if not self._name or not _re_arg.match(name):
            raise RuntimeError('Argument name not provided or has invalid format: ' + str(self._name))

        # Private member for properties
        self.__constructor = None
        self.__declaration = None
        self.__member_name = '_' + self._name

    @property
    def constructor(self):
        if not self.__constructor:
            self.__constructor = self.member_name + '(' + self._name + ')'

        return self.__constructor

    @property
    def declaration(self):
        if not self.__declaration:
            self.__declaration = 'std::string const ' + self.member_name + ';'

        return self.__declaration

    @property
    def member_name(self):
        return self.__member_name


class Commented(object):
    """Represents a commentable element.
    """
    def __init__(self, comment=None, **kwargs):
        super(Commented, self).__init__()

        self._comment = comment
        if self._comment and not _re_comment.match(self._comment):
            raise RuntimeError('Comment has invalid format: ' + str(self._comment))

        if len(kwargs) != 0:
            raise RuntimeError('Extraneous keys detected: ' + str(kwargs.keys()))

    @property
    def is_commented(self):
        return bool(self._comment)


class Variable(Commented):
    """Represents a class member variable in a model. Primarily, this gives
    information about the underlying type of a parameter or state field.
    """
    def __init__(self, name=None, type=None, **kwargs):
        super(Variable, self).__init__(**kwargs)

        self._name = name
        if not self._name or not _re_variable.match(self._name):
            raise RuntimeError('Name not provide or has invalid format: ' + str(self._name))

        self._type = type
        if not self._type and not _re_type.match(self._type):
            raise RuntimeError('Type not provided or has invalid format: ' + str(self._type))

        # Private member for properties
        self.__member_name = None
        self.__string_name = None

        # Underlying type
        underlying_type_matches = 0
        self.__underlying_type = None
        for underlying_type in ['Integer', 'Real', 'Vector2', 'Vector3', 'Vector4']:
            if underlying_type in self._type:
                underlying_type_matches = underlying_type_matches + 1
                self.__underlying_type = underlying_type

        if underlying_type_matches != 1:
            raise RuntimeError('Multiple or no underlying types specified: ' + str(self._type))

    @property
    def member_name(self):
        if not self.__member_name:
            self.__member_name = self._name.replace('{', '').replace('}', '').replace('.', '_')

        return self.__member_name

    @property
    def string_name(self):
        if not self.__string_name:
            self.__string_name = '"' + self._name + '"'
            for match in _re_replace.findall(self._name):
                self.__string_name = self.__string_name.replace(match, '"+_' + match[1:-1] + '+"')

            if self.__string_name.startswith('""+'):
                self.__string_name = self.__string_name[3:]
            if self.__string_name.endswith('+""'):
                self.__string_name = self.__string_name[:-3]

        return self.__string_name

    @property
    def underlying_type(self):
        return self.__underlying_type


class Parameter(Variable):
    """Represents a model parameter.
    """
    def __init__(self, **kwargs):
        super(Parameter, self).__init__(**kwargs)

        # Private member for properties
        self.__constructor = None
        self.__declaration = None

    @property
    def constructor(self):
        if not self.__constructor:
            self.__constructor = self.member_name + '(config[' + self.string_name + '].template get<' + self.underlying_type + '>())'

        return self.__constructor

    @property
    def declaration(self):
        if not self.__declaration:
            self.__declaration = 'Parameter<' + self.underlying_type + '> const ' + self.member_name + ';'

        return self.__declaration


class StateField(Variable):
    """Represents a state field.
    """
    def __init__(self, **kwargs):
        super(StateField, self).__init__(**kwargs)

        self._type_modifiers = set(_re_modifier.findall(self._type))

        # Private members for properties
        self.__is_writable = 'Writable' in self._type_modifiers

    @property
    def is_writable(self):
        return self.__is_writable


class AddsStateField(StateField):
    """Represents a state field added to the simulation by the model.
    """
    def __init__(self, **kwargs):
        super(AddsStateField, self).__init__(**kwargs)

        # Private member for properties
        self.__adds_expression = None
        self.__constructor = None
        self.__declaration = None
        self.__is_initialized = 'Initialized' in self._type_modifiers
        self.__is_lazy = 'Lazy' in self._type_modifiers
        self.__is_writable = 'Writable' in self._type_modifiers

        _type_modifiers = set(self._type_modifiers)

        _type_modifiers.remove(self.underlying_type)
        if self.is_initialized:
            _type_modifiers.remove('Initialized')
        if self.is_lazy:
            _type_modifiers.remove('Lazy')
        if self.is_writable:
            _type_modifiers.remove('Writable')

        if len(_type_modifiers) != 0:
            raise RuntimeError('Invalid adds type modifiers: ' + str(_type_modifiers))

        if self.is_lazy and (self.is_initialized or self.is_writable):
            raise RuntimeError('A lazy field cannot be initialized or writable: ' + str(self._type))

    @property
    def adds_expression(self):
        if not self.__adds_expression:
            if self.is_writable:
                self.__adds_expression = 'state.add_writable(&' + self.member_name + ');'
            else:
                self.__adds_expression = 'state.add(&' + self.member_name + ');'

        return self.__adds_expression

    @property
    def constructor(self):
        if not self.__constructor:
            if self.is_lazy:
                self.__constructor = self.member_name + '(' + self.string_name + ', std::bind(&D::' + self.member_name + ', &derived()))'
            else:
                if self.is_initialized:
                    self.__constructor = self.member_name + '(' + self.string_name + ', config[' + self.string_name + '].template get<' + self.underlying_type + '>())'
                else:
                    self.__constructor = self.member_name + '(' + self.string_name + ')'

        return self.__constructor

    @property
    def declaration(self):
        if not self.__declaration:
            if self.__is_lazy:
                self.__declaration = 'StateFieldLazy<' + self.underlying_type + '> ' + self.member_name + ';'
            else:
                self.__declaration = 'StateFieldValued<' + self.underlying_type + '> ' + self.member_name + ';'

        return self.__declaration

    @property
    def is_initialized(self):
        return self.__is_initialized

    @property
    def is_lazy(self):
        return self.__is_lazy


class GetsStateField(StateField):
    """Represents a state field retrieved from the simulation by the model.
    """
    def __init__(self, **kwargs):
        super(GetsStateField, self).__init__(**kwargs)

        # Private member for properties
        self.__constructor = None
        self.__declaration = None
        self.__gets_expression = None

        _type_modifiers = self._type_modifiers

        _type_modifiers.remove(self.underlying_type)
        if self.is_writable:
            _type_modifiers.remove('Writable')
        
        if len(_type_modifiers) != 0:
            raise RuntimeError('Invalid gets type modifiers: ' + str(_type_modifiers))

    @property
    def constructor(self):
        if not self.__constructor:
            self.__constructor = self.member_name + '(nullptr)'

        return self.__constructor

    @property
    def declaration(self):
        if not self.__declaration:
            if self.is_writable:
                self.__declaration = 'StateFieldWritable<' + self.underlying_type + '> *' + self.member_name + ';'
            else:
                self.__declaration = 'StateField<' + self.underlying_type + '> const *' + self.member_name + ';'

        return self.__declaration

    @property
    def gets_expression(self):
        if not self.__gets_expression:
            if self.is_writable:
                self.__gets_expression = self.member_name + ' = get_writable_field<' + self.underlying_type + '>(state, ' + self.string_name + ');'
            else:
                self.__gets_expression = self.member_name + ' = get_field<' + self.underlying_type + '>(state, ' + self.string_name + ');'

        return self.__gets_expression


class Model(Commented):
    """Represents a model.
    """
    def __init__(self, name=None, type=None, args=[], params=[], adds=[], gets=[], **kwargs):
        super(Model, self).__init__(**kwargs)

        self._name = name
        if not self._name or not _re_model.match(self._name):
            raise RuntimeError('Model name not provide or has invalid format: ' + str(self._name))

        self._type = type
        if not self._type or self._type != 'Model':
            raise RuntimeError('Model type not provided or has invalid format: ' + str(self._type))

        self._args = [Argument(arg) for arg in args]
        self._params = [Parameter(**param) for param in params]
        self._adds = [AddsStateField(**add) for add in adds]
        self._gets = [GetsStateField(**get) for get in gets]

        # Private member for properties
        self.__code = None

    @property
    def code(self):
        if not self.__code:
            self.__code = ''

            self.__code += \
            '/* This file has been autogenerated. Do not edit manually. */\n' + \
            '\n' + \
            '#ifndef PSIM_AUTOCODED_{}_HPP_\n'.format(self._name.upper()) + \
            '#define PSIM_AUTOCODED_{}_HPP_\n'.format(self._name.upper()) + \
            '\n' + \
            '#include <psim/core/configuration.hpp>\n' + \
            '#include <psim/core/model.hpp>\n' + \
            '#include <psim/core/parameter.hpp>\n' + \
            '#include <psim/core/state.hpp>\n' + \
            '#include <psim/core/state_field_lazy.hpp>\n' + \
            '#include <psim/core/state_field_valued.hpp>\n' + \
            '#include <psim/core/types.hpp>\n' + \
            '\n' + \
            '#include <functional>\n' + \
            '\n' + \
            'namespace psim {\n' + \
            '\n' +\
            'template <class D>\n' + \
            'class {0} : public {1}'.format(self._name, self._type) + ' {\n' + \
            ' private:\n'

            # Create member variables for all args
            for arg in self._args:
                self.__code += '  ' + arg.declaration + '\n'

            self.__code += \
            '\n' + \
            '  D const &derived() const {\n' + \
            '    return static_cast<D const &>(*this);\n' + \
            '  }\n' + \
            '\n' + \
            '  D &derived() {\n' + \
            '    return static_cast<D &>(*this);\n' + \
            '  }\n' + \
            '\n' + \
            ' protected:\n'

            # Member variables for all parameters
            if len(self._params) > 0:
                for param in self._params:
                    self.__code += '  ' + param.declaration + '\n'
                self.__code += '\n'

            # Member variables for adds fields
            if len(self._adds) > 0:
                for add in self._adds:
                    self.__code += '  ' + add.declaration + '\n'
                self.__code += '\n'

            # Member variables for gets fields
            if len(self._gets) > 0:
                for get in self._gets:
                    self.__code += '  ' + get.declaration + '\n'
                self.__code += '\n'

            self.__code += \
            ' public:\n' + \
            '  {}() = delete;\n'.format(self._name) + \
            '\n' + \
            '  virtual ~{}() = default;\n'.format(self._name) + \
            '\n' + \
            '  {}(Configuration const &config'.format(self._name)

            # Arg arguments
            for arg in self._args:
                self.__code += ', std::string const &' + arg._name

            self.__code += ')\n' + \
            '  : {}()'.format(self._type)

            # Construct member varaibles
            for member in itertools.chain(self._args, self._params, self._adds, self._gets):
                self.__code += ',\n    ' + member.constructor

            self.__code += '\n' + \
            '  { }\n' + \
            '\n' + \
            '  virtual void add_fields(State &state) override {\n' + \
            '    this->{}::add_fields(state);\n'.format(self._type) + \
            '\n'

            # Add fields to the simulation
            for add in self._adds:
                self.__code += '    ' + add.adds_expression + '\n'

            self.__code += \
            '  }\n' + \
            '\n' + \
            '  virtual void get_fields(State &state) override {\n' + \
            '    this->{}::get_fields(state);\n'.format(self._type) + \
            '\n'

            # Get fields from the simulation
            for get in self._gets:
                self.__code += '    ' + get.gets_expression + '\n'

            self.__code += \
            '  }\n' + \
            '\n' + \
            '  virtual void step() override {\n' + \
            '    this->{}::step();\n'.format(self._type) + \
            '\n'

            # Reset all lazy fields
            for add in self._adds:
                if add.is_lazy:
                    self.__code += '    ' + add.member_name + '.reset();\n'

            self.__code += \
            '  }\n' + \
            '};\n' + \
            '} // namespace psim\n' + \
            '\n' + \
            '#endif\n'

        return self.__code


def main(args):
    data = None
    with open(args[1], 'r') as istream:
        data = yaml.safe_load(istream)

    if type(data) is not dict:
        raise RuntimeError('Loading YAML file did not result in a dictionary: ' + args[1])

    with open(args[2], 'w') as ostream:
        ostream.write(Model(**data).code)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
